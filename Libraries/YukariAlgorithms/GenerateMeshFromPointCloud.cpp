/** @file */

#include "GenerateMeshFromPointCloud.h"

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

#include <YukariCloudCapture/ICloudGrabber.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  GenerateMeshFromPointCloud::GenerateMeshFromPointCloud()
      : m_logger(Common::LoggingService::GetLogger("GenerateMeshFromPointCloud"))
  {
    /* Add default validator */
    m_validator = [](const IAlgorithm &alg) {
      /* Check that at least one point cloud was provided */
      Property_sptr cloud = alg.getProperty(Processing::INPUT, "cloud");
      if (cloud->size() == 0)
        return "Must provide at least one point clouds";

      return "";
    };
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

  void GenerateMeshFromPointCloud::doExecute()
  {
    Property_sptr cloud = getProperty(Processing::INPUT, "cloud");
    size_t len = cloud->size();
    Property_sptr mesh = std::make_shared<Property>(len);

    GenerateMeshFromPointCloud::Parameters params;

    for (size_t i = 0; i < len; i++)
    {
      m_logger->debug("Mesh estimation for cloud {}", i);
      (*mesh)[i] = estimateSingle(cloud->value<ICloudGrabber::Cloud::ConstPtr>(i), params);
    }

    setProperty(Processing::OUTPUT, "mesh", mesh);
  }
}
}
