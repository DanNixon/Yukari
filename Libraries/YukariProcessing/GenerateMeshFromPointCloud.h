/** @file */

#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class GenerateMeshFromPointCloud
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    struct Parameters
    {
      /* TODO */
    };

  public:
    GenerateMeshFromPointCloud();

    pcl::PolygonMesh::Ptr estimateSingle(const CloudConstPtr cloud, const Parameters &params)
    {
      /* Normal estimation */
      m_logger->trace("Normal estimation");
      pcl::NormalEstimation<POINT_TYPE, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      typename pcl::search::KdTree<POINT_TYPE>::Ptr tree(new pcl::search::KdTree<POINT_TYPE>);
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

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
