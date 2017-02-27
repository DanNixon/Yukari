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

      Property inputIMUFrames(5);
      inputIMUFrames[0] =
          std::make_shared<IMUFrame>(IMUFrame::Duration(20.0f), Quaternion(), Vector3());
      inputIMUFrames[1] =
          std::make_shared<IMUFrame>(IMUFrame::Duration(21.0f), Quaternion(), Vector3());
      inputIMUFrames[2] =
          std::make_shared<IMUFrame>(IMUFrame::Duration(22.0f), Quaternion(), Vector3());
      inputIMUFrames[3] =
          std::make_shared<IMUFrame>(IMUFrame::Duration(19.0f), Quaternion(), Vector3());
      inputIMUFrames[4] =
          std::make_shared<IMUFrame>(IMUFrame::Duration(23.0f), Quaternion(), Vector3());

      alg.setProperty(Processing::INPUT, "frames", inputIMUFrames);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property results = alg.getProperty(OUTPUT, "transformation");
      BOOST_CHECK_EQUAL(results.size(), inputIMUFrames.size());

      /* BOOST_CHECK_EQUAL(results.value<int>(0), 31); */
    }
  }
}
}

#endif
