/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "RemoveNaNFromPointCloudTest"

#include <YukariCloudCapture/ICloudGrabber.h>
#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/AlgorithmFactory.h>
#include <YukariAlgorithms/RemoveNaNFromPointCloud.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(RemoveNaNFromPointCloud_Create)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("RemoveNaNFromPointCloud");
      BOOST_CHECK(alg);
    }

    BOOST_AUTO_TEST_CASE(RemoveNaNFromPointCloud_Execute)
    {
      RemoveNaNFromPointCloud alg;

      Property_sptr cloud = std::make_shared<Property>(4);

      /* Point cloud with no NaN values */
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 2;
        c->height = 2;
        c->is_dense = false;
        c->points.resize(4);

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

        (*cloud)[0] = c;
      }

      /* Point cloud with a single NaN value */
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 2;
        c->height = 2;
        c->is_dense = false;
        c->points.resize(4);

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

        (*cloud)[1] = c;
      }

      /* Point cloud with a point of all NaN values */
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 2;
        c->height = 2;
        c->is_dense = false;
        c->points.resize(4);

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

        (*cloud)[2] = c;
      }

      /* Point cloud containing all NaNs */
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 2;
        c->height = 2;
        c->is_dense = false;
        c->points.resize(4);

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

        (*cloud)[3] = c;
      }

      alg.setProperty(Processing::INPUT, "cloud", cloud);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property_sptr results = alg.getProperty(Processing::OUTPUT, "cloud");
      BOOST_CHECK_EQUAL(results->size(), cloud->size());

      /* No NaNs so all data should be identical */
      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(0);
        BOOST_CHECK_EQUAL(c->points.size(), 4);
      }

      /* Single point has NaN values */
      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(1);
        BOOST_CHECK_EQUAL(c->points.size(), 3);
      }

      /* Single point has NaN values */
      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(2);
        BOOST_CHECK_EQUAL(c->points.size(), 3);
      }

      /* All points had NaN values */
      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(3);
        BOOST_CHECK_EQUAL(c->points.size(), 0);
      }
    }
  }
}
}

#endif
