/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>
#include <sstream>

#include <YukariIMU/IMUFrame.h>

using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(IMUFrameTest)

    BOOST_AUTO_TEST_CASE(IMUFrame_Stream_Out_Empty)
    {
      IMUFrame f(IMUFrame::Duration(250.0f));

      std::stringstream str;
      str << f;

      BOOST_CHECK_EQUAL(str.str(), "(dt=250, o=[1, 0, 0, 0], p=[0, 0, 0])");
    }

    BOOST_AUTO_TEST_CASE(IMUFrame_Stream_Out)
    {
      IMUFrame f(IMUFrame::Duration(250.0f));
      f.orientation() = Eigen::Quaternionf(Vector3(30.0f, 30.0f, 30.0f), Quaternion::ZYX, DEGREES);
      f.position() = Vector3(5.5f, 7.8f, 2.1f);

      std::stringstream str;
      str << f;

      BOOST_CHECK_EQUAL(str.str(),
                        "(dt=250, o=[0.918559, 0.176777, 0.306186, 0.176777], p=[5.5, 7.8, 2.1])");
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
