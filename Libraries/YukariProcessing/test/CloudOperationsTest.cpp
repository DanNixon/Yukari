#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariProcessing/CloudOperations.h>

namespace Yukari
{
namespace Processing
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(CloudOperationsTest)

    BOOST_AUTO_TEST_CASE(CloudOperations_ApplyTransformationToCloud)
    {
      typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

      /* Cloud */
      typename Cloud::Ptr c(new Cloud());

      c->width = 1;
      c->height = 1;
      c->is_dense = false;
      c->points.resize(1);

      c->points[0].x = 1.0f;
      c->points[0].y = 1.0f;
      c->points[0].z = 1.0f;

      c->points[0].rgba = 0xFFFFFFFF;

      /* Transformation */
      Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
      t.col(3) << 2.0f, 8.0f, 5.0f, 1.0f;

      auto c2 = CloudOperations<pcl::PointXYZRGBA>::ApplyTransformationToCloud(c, t);

      auto p = c2->points[0];
      BOOST_CHECK_EQUAL(p.x, 3.0f);
      BOOST_CHECK_EQUAL(p.y, 9.0f);
      BOOST_CHECK_EQUAL(p.z, 6.0f);
    }

    BOOST_AUTO_TEST_CASE(CloudOperations_ConcatenateClouds)
    {
      typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

      std::vector<typename Cloud::Ptr> clouds;

      for (size_t i = 0; i < 5; i++)
      {
        typename Cloud::Ptr c(new Cloud());

        c->width = 1;
        c->height = 1;
        c->is_dense = false;
        c->points.resize(1);

        c->points[0].x = (float)i;
        c->points[0].y = (float)i;
        c->points[0].z = (float)i;

        c->points[0].rgba = 0xFFFFFFFF;

        clouds.push_back(c);
      }

      auto resultCloud = CloudOperations<pcl::PointXYZRGBA>::ConcatenateClouds(clouds);

      BOOST_CHECK_EQUAL(resultCloud->width, 5);
      BOOST_CHECK_EQUAL(resultCloud->height, 1);
      BOOST_CHECK_EQUAL(resultCloud->points.size(), clouds.size());

      BOOST_CHECK_EQUAL(resultCloud->points[0].x, 0.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[1].x, 1.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[2].x, 2.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[3].x, 3.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[4].x, 4.0f);
    }

    BOOST_AUTO_TEST_CASE(CloudOperations_RemoveNaNFromCloud_1)
    {
      typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

      typename Cloud::Ptr c(new Cloud());

      c->width = 2;
      c->height = 2;
      c->is_dense = false;
      c->points.resize(4);

      /* Point cloud with no NaN values */
      c->points[0].x = 1.0f;
      c->points[0].y = 1.0f;
      c->points[0].z = 1.0f;

      c->points[1].x = 1.0f;
      c->points[1].y = 1.0f;
      c->points[1].z = 1.0f;

      c->points[2].x = 1.0f;
      c->points[2].y = 1.0f;
      c->points[2].z = 1.0f;

      c->points[3].x = 1.0f;
      c->points[3].y = 1.0f;
      c->points[3].z = 1.0f;

      for (size_t i = 0; i < 4; i++)
        c->points[i].rgba = 0xFFFFFFFF;

      auto c2 = CloudOperations<pcl::PointXYZRGBA>::RemoveNaNFromCloud(c);

      /* No NaNs so all data should be identical */
      BOOST_CHECK_EQUAL(c2->points.size(), 4);
    }

    BOOST_AUTO_TEST_CASE(CloudOperations_RemoveNaNFromCloud_2)
    {
      typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

      typename Cloud::Ptr c(new Cloud());

      c->width = 2;
      c->height = 2;
      c->is_dense = false;
      c->points.resize(4);

      /* Point cloud with a single NaN value */
      c->points[0].x = 1.0f;
      c->points[0].y = 1.0f;
      c->points[0].z = 1.0f;

      c->points[1].x = 1.0f;
      c->points[1].y = 1.0f;
      c->points[1].z = 1.0f;

      c->points[2].x = std::numeric_limits<float>::quiet_NaN();
      c->points[2].y = 1.0f;
      c->points[2].z = 1.0f;

      c->points[3].x = 1.0f;
      c->points[3].y = 1.0f;
      c->points[3].z = 1.0f;

      for (size_t i = 0; i < 4; i++)
        c->points[i].rgba = 0xFFFFFFFF;

      auto c2 = CloudOperations<pcl::PointXYZRGBA>::RemoveNaNFromCloud(c);

      /* Single point has NaN values */
      BOOST_CHECK_EQUAL(c2->points.size(), 3);
    }

    BOOST_AUTO_TEST_CASE(CloudOperations_RemoveNaNFromCloud_3)
    {
      typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

      typename Cloud::Ptr c(new Cloud());

      c->width = 2;
      c->height = 2;
      c->is_dense = false;
      c->points.resize(4);

      /* Point cloud with a point of all NaN values */
      c->points[0].x = 1.0f;
      c->points[0].y = 1.0f;
      c->points[0].z = 1.0f;

      c->points[1].x = std::numeric_limits<float>::quiet_NaN();
      c->points[1].y = std::numeric_limits<float>::quiet_NaN();
      c->points[1].z = std::numeric_limits<float>::quiet_NaN();

      c->points[2].x = 1.0f;
      c->points[2].y = 1.0f;
      c->points[2].z = 1.0f;

      c->points[3].x = 1.0f;
      c->points[3].y = 1.0f;
      c->points[3].z = 1.0f;

      for (size_t i = 0; i < 4; i++)
        c->points[i].rgba = 0xFFFFFFFF;

      auto c2 = CloudOperations<pcl::PointXYZRGBA>::RemoveNaNFromCloud(c);

      /* Single point has NaN values */
      BOOST_CHECK_EQUAL(c2->points.size(), 3);
    }

    BOOST_AUTO_TEST_CASE(CloudOperations_RemoveNaNFromCloud_4)
    {
      typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

      typename Cloud::Ptr c(new Cloud());

      c->width = 2;
      c->height = 2;
      c->is_dense = false;
      c->points.resize(4);

      /* Point cloud containing all NaNs */
      c->points[0].x = std::numeric_limits<float>::quiet_NaN();
      c->points[0].y = std::numeric_limits<float>::quiet_NaN();
      c->points[0].z = std::numeric_limits<float>::quiet_NaN();

      c->points[1].x = std::numeric_limits<float>::quiet_NaN();
      c->points[1].y = std::numeric_limits<float>::quiet_NaN();
      c->points[1].z = std::numeric_limits<float>::quiet_NaN();

      c->points[2].x = std::numeric_limits<float>::quiet_NaN();
      c->points[2].y = std::numeric_limits<float>::quiet_NaN();
      c->points[2].z = std::numeric_limits<float>::quiet_NaN();

      c->points[3].x = std::numeric_limits<float>::quiet_NaN();
      c->points[3].y = std::numeric_limits<float>::quiet_NaN();
      c->points[3].z = std::numeric_limits<float>::quiet_NaN();

      for (size_t i = 0; i < 4; i++)
        c->points[i].rgba = 0xFFFFFFFF;

      auto c2 = CloudOperations<pcl::PointXYZRGBA>::RemoveNaNFromCloud(c);

      /* All points had NaN values */
      BOOST_CHECK_EQUAL(c2->points.size(), 0);
    }

    BOOST_AUTO_TEST_SUITE_END()
  };
}
}

#endif
