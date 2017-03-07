/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariIMU/ExtendedIMUFrame.h>
#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(ExtendedIMUFrameTest)

    BOOST_AUTO_TEST_CASE(ExtendedIMUFrame_Stream_Out_Empty)
    {
      ExtendedIMUFrame f(IMUFrame::Duration(250.0f));

      std::stringstream str;
      str << f;

      BOOST_CHECK_EQUAL(str.str(),
                        "(dt=250, o=[1, 0, 0, 0], p=[0, 0, 0], av=[0, 0, 0], lv=[0, 0, 0])");
    }

    BOOST_AUTO_TEST_CASE(ExtendedIMUFrame_Stream_Out)
    {
      ExtendedIMUFrame f(IMUFrame::Duration(250.0f));
      f.orientation() = Quaternion(Vector3(30.0f, 30.0f, 30.0f), Quaternion::ZYX, DEGREES);
      f.position() = Vector3(5.5f, 7.8f, 2.1f);
      f.angularVelocity() = Vector3(8.7f, 2.4f, 9.8f);
      f.linearVelocity() = Vector3(1.7f, 0.5f, 9.1f);

      std::stringstream str;
      str << f;

      BOOST_CHECK_EQUAL(str.str(), "(dt=250, o=[0.918559, 0.176777, 0.306186, 0.176777], p=[5.5, "
                                   "7.8, 2.1], av=[8.7, 2.4, 9.8], lv=[1.7, 0.5, 9.1])");
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
