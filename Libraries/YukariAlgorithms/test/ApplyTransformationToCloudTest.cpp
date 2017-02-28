/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "ApplyTransformationToCloudTest"

#include <Eigen/Geometry>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/ApplyTransformationToCloud.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(ApplyTransformationToCloud_Execute)
    {
      const size_t len = 3;

      ApplyTransformationToCloud alg;

      Property_sptr cloud = std::make_shared<Property>(len);
      Property_sptr transform = std::make_shared<Property>(len);

      for (size_t i = 0; i < len; i++)
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 1;
        c->height = 1;
        c->is_dense = false;
        c->points.resize(1);

        c->points[0].x = 1.0f;
        c->points[0].y = 1.0f;
        c->points[0].z = 1.0f;

        c->points[0].rgba = 0xFFFFFFFF;

        (*cloud)[i] = c;
      }

      {
        Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
        t.col(3) << 2.0f, 8.0f, 5.0f, 1.0f;
        (*transform)[0] = t;
      }

      {
        Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
        t.diagonal() << 2.0f, 8.0f, 5.0f, 1.0f;
        (*transform)[1] = t;
      }

      {
        Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
        Eigen::AngleAxis<float> rot(boost::math::constants::pi<float>(),
                                    Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        t.block(0, 0, 3, 3) = rot.matrix();
        (*transform)[2] = t;
      }

      alg.setProperty(Processing::INPUT, "cloud", cloud);
      alg.setProperty(Processing::INPUT, "transform", transform);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property_sptr results = alg.getProperty(Processing::OUTPUT, "cloud");
      BOOST_CHECK_EQUAL(results->size(), len);

      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(0);
        auto p = c->points[0];
        BOOST_CHECK_EQUAL(p.x, 3.0f);
        BOOST_CHECK_EQUAL(p.y, 9.0f);
        BOOST_CHECK_EQUAL(p.z, 6.0f);
      }

      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(1);
        auto p = c->points[0];
        BOOST_CHECK_EQUAL(p.x, 2.0f);
        BOOST_CHECK_EQUAL(p.y, 8.0f);
        BOOST_CHECK_EQUAL(p.z, 5.0f);
      }

      {
        ICloudGrabber::Cloud::Ptr c = results->value<ICloudGrabber::Cloud::Ptr>(2);
        auto p = c->points[0];
        BOOST_CHECK_EQUAL(p.x, 1.0f);
        BOOST_CHECK_CLOSE(p.y, -1.0f, 0.01f);
        BOOST_CHECK_CLOSE(p.z, -1.0f, 0.01f);
      }
    }
  }
}
}

#endif
