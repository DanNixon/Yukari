/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "IMUFrameToEigenTransformationTest"

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/IMUFrameToEigenTransformation.h>
#include <YukariIMU/IMUFrame.h>

using namespace Yukari::Processing;
using namespace Yukari::Maths;
using namespace Yukari::IMU;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(IMUFrameToEigenTransformation_Execute)
    {
      IMUFrameToEigenTransformation alg;

      Property_sptr inputIMUFrames = std::make_shared<Property>(5);
      (*inputIMUFrames)[0] = std::make_shared<IMUFrame>(
          IMUFrame::Duration(20.0f), Quaternion(Vector3(0.5f, 0.6f, 0.5f), 20.0f, DEGREES),
          Vector3(1.0f, 2.0f, 3.0f));
      (*inputIMUFrames)[1] = std::make_shared<IMUFrame>(
          IMUFrame::Duration(21.0f), Quaternion(Vector3(0.5f, 0.6f, 0.4f), 20.0f, DEGREES),
          Vector3(4.0f, 5.0f, 6.0f));
      (*inputIMUFrames)[2] = std::make_shared<IMUFrame>(
          IMUFrame::Duration(22.0f), Quaternion(Vector3(0.5f, 0.6f, 0.3f), 20.0f, DEGREES),
          Vector3(7.0f, 8.0f, 9.0f));
      (*inputIMUFrames)[3] = std::make_shared<IMUFrame>(
          IMUFrame::Duration(19.0f), Quaternion(Vector3(0.5f, 0.6f, 0.3f), 15.0f, DEGREES),
          Vector3(10.0f, 11.0f, 12.0f));
      (*inputIMUFrames)[4] = std::make_shared<IMUFrame>(
          IMUFrame::Duration(23.0f), Quaternion(Vector3(0.8f, 0.6f, 0.5f), 30.0f, DEGREES),
          Vector3(13.0f, 14.0f, 15.0f));

      alg.setProperty(Processing::INPUT, "frames", inputIMUFrames);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property_sptr results = alg.getProperty(OUTPUT, "transformation");
      BOOST_CHECK_EQUAL(results->size(), inputIMUFrames->size());

      {
        Eigen::Matrix4f m;
        // clang-format off
        m <<  0.95722f, -0.16337f,  0.23882f, 1.0f,
              0.20544f,  0.96494f, -0.16337f, 2.0f,
             -0.20375f,  0.20544f,  0.95722f, 3.0f,
              0.0f    ,  0.0f    ,  0.0f    , 1.0f;
        // clang-format on
        BOOST_CHECK(results->value<Eigen::Matrix4f>(0).isApprox(m));
      }

      {
        Eigen::Matrix4f m;
        // clang-format off
        m <<  0.95927f, -0.13241f,  0.24953f, 4.0f,
              0.17940f,  0.96789f, -0.17609f, 5.0f,
             -0.21820f,  0.21368f,  0.95222f, 6.0f,
              0.0f    ,  0.0f    ,  0.0f    , 1.0f;
        // clang-format on
        BOOST_CHECK(results->value<Eigen::Matrix4f>(1).isApprox(m));
      }

      {
        Eigen::Matrix4f m;
        // clang-format off
        m <<  0.96123f, -0.09679f,  0.25820f, 7.0f,
              0.14848f,  0.97071f, -0.18889f, 8.0f,
             -0.23235f,  0.21990f,  0.94745f, 9.0f,
              0.0f    ,  0.0f    ,  0.0f    , 1.0f;
        // clang-format on
        BOOST_CHECK(results->value<Eigen::Matrix4f>(2).isApprox(m));
      }

      {
        Eigen::Matrix4f m;
        // clang-format off
        m <<  0.97810f, -0.07820f,  0.19291f, 10.0f,
              0.10741f,  0.98345f, -0.14591f, 11.0f,
             -0.17831f,  0.16344f,  0.97031f, 12.0f,
              0.0f    ,  0.0f    ,  0.0f    , 1.0f;
        // clang-format on
        BOOST_CHECK(results->value<Eigen::Matrix4f>(3).isApprox(m));
      }

      {
        Eigen::Matrix4f m;
        // clang-format off
        m <<  0.93462f, -0.17216f,  0.31120f, 13.0f,
              0.27505f,  0.90461f, -0.32562f, 14.0f,
             -0.22546f,  0.38992f,  0.89282f, 15.0f,
              0.0f    ,  0.0f    ,  0.0f    , 1.0f;
        // clang-format on
        BOOST_CHECK(results->value<Eigen::Matrix4f>(4).isApprox(m));
      }
    }
  }
}
}

#endif
