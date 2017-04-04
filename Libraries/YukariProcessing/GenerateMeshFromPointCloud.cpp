/** @file */

#include "GenerateMeshFromPointCloud.h"

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

#include <YukariCloudCapture/ICloudGrabber.h>

using namespace Yukari::CloudCapture;

namespace Yukari
{
namespace Processing
{
  GenerateMeshFromPointCloud::GenerateMeshFromPointCloud()
      : m_logger(Common::LoggingService::GetLogger("GenerateMeshFromPointCloud"))
  {
  }

  pcl::PolygonMesh::Ptr
  GenerateMeshFromPointCloud::estimateSingle(const ICloudGrabber::Cloud::ConstPtr cloud,
                                             const GenerateMeshFromPointCloud::Parameters &params)
  {
    /* Normal estimation */
    m_logger->trace("Normal estimation");
    pcl::NormalEstimation<ICloudGrabber::PointType, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<ICloudGrabber::PointType>::Ptr tree(
        new pcl::search::KdTree<ICloudGrabber::PointType>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    /* Concatenate the XYZ and normal fields  */
    m_logger->trace("Cloud concatenation");
    pcl::PointCloud<pcl::PointXYZ> c2;
    pcl::copyPointCloud(*cloud, c2);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(c2, *normals, *cloud_with_normals);

    /* Create search tree */
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    /* Set typical values for the parameters */
    gp3.setSearchRadius(0.025);
    gp3.setMu(0.1);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    /* Get results */
    m_logger->trace("Mesh reconstruction");
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*triangles);

    m_logger->trace("Cloud processed");

    return triangles;
  }
}
}
