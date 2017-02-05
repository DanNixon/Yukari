/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "QuaternionTest"

#include <boost/test/unit_test.hpp>

#include <YukariMaths/Quaternion.h>

#define TOL 0.00001f

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    /* Quaternion functionality */

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Empty)
    {
      Quaternion q;
      BOOST_CHECK_EQUAL(q.i(), 0.0f);
      BOOST_CHECK_EQUAL(q.j(), 0.0f);
      BOOST_CHECK_EQUAL(q.k(), 0.0f);
      BOOST_CHECK_EQUAL(q.w(), 1.0f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Value)
    {
      Quaternion q(0.354f, 0.354f, 0.146f, 0.854f);
      BOOST_CHECK_EQUAL(q.i(), 0.354f);
      BOOST_CHECK_EQUAL(q.j(), 0.354f);
      BOOST_CHECK_EQUAL(q.k(), 0.146f);
      BOOST_CHECK_EQUAL(q.w(), 0.854f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Axis_Angle_Degrees)
    {
      Vector3 axis(1.0f, 1.0f, 1.0f);
      Quaternion q(axis, 90.0f, DEGREES);

      double c = M_SQRT1_2;
      double s = c / std::sqrt(3.0);
      BOOST_CHECK_CLOSE(q[0], s, TOL);
      BOOST_CHECK_CLOSE(q[1], s, TOL);
      BOOST_CHECK_CLOSE(q[2], s, TOL);
      BOOST_CHECK_CLOSE(q[3], c, TOL);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Euler_Degrees)
    {
      Quaternion q(15.0f, 45.0f, 55.0f, DEGREES);
      BOOST_CHECK_EQUAL(q.i(), 0.392f);
      BOOST_CHECK_EQUAL(q.j(), 0.379f);
      BOOST_CHECK_EQUAL(q.k(), 0.282f);
      BOOST_CHECK_EQUAL(q.w(), 0.789f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Inverse)
    {
      Quaternion q(5.0, 2.0, 4.5, 8.9);
      Quaternion inv = q.inverse();
      Quaternion r = q * inv;
      BOOST_CHECK_CLOSE(r.w(), 1.0f, TOL);
      BOOST_CHECK_CLOSE(r.i(), 0.0f, TOL);
      BOOST_CHECK_CLOSE(r.j(), 0.0f, TOL);
      BOOST_CHECK_CLOSE(r.k(), 0.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Multiply)
    {
      Quaternion q1(1, 1, 1, 1);
      Quaternion q2(2, 1, 3, -1);
      Quaternion q3 = q1 * q2;
      BOOST_CHECK_EQUAL(q3.i(), -1.0f);
      BOOST_CHECK_EQUAL(q3.j(), 1.0f);
      BOOST_CHECK_EQUAL(q3.k(), 3.0f);
      BOOST_CHECK_EQUAL(q3.w(), -7.0f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Multiply_Assignment)
    {
      Quaternion q1(1, 1, 1, 1);
      Quaternion q2(2, 1, 3, -1);
      q1 *= q2;
      BOOST_CHECK_EQUAL(q1.i(), -1.0f);
      BOOST_CHECK_EQUAL(q1.j(), 1.0f);
      BOOST_CHECK_EQUAL(q1.k(), 3.0f);
      BOOST_CHECK_EQUAL(q1.w(), -7.0f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_To_Euler)
    {
      Quaternion x(Vector3(1, 0, 0), 120.0f, DEGREES);
      Quaternion y(Vector3(0, 1, 0), -60.0f, DEGREES);
      Quaternion z(Vector3(0, 0, 1), 90.0f, DEGREES);
      Quaternion q = x * y * z;
      Vector3 e = q.toEulerAngles(DEGREES);
      BOOST_CHECK_EQUAL(e.x(), 120.0f);
      BOOST_CHECK_EQUAL(e.y(), -60.0f);
      BOOST_CHECK_EQUAL(e.z(), 90.0);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Rotate_Vector)
    {
      Quaternion q(Vector3(0.0f, 1.0f, 0.0f), 90.0f, DEGREES);
      Vector3 v(1.0f, 0.0f, 0.0f);
      v = q.rotate(v);
      BOOST_CHECK_CLOSE(v.x(), 0.0f, TOL);
      BOOST_CHECK_CLOSE(v.y(), 0.0f, TOL);
      BOOST_CHECK_CLOSE(v.z(), -1.0f, TOL);
    }

    /* BaseMathType functionality */

    BOOST_AUTO_TEST_CASE(Quaternion_Equality)
    {
      Quaternion q1(1.1f, 2.2f, 3.3f, 4.4f);
      Quaternion q2(4.4f, 6.6f, 7.7f, 8.8f);
      Quaternion q3(1.1f, 2.2f, 3.3f, 4.4f);
      BOOST_CHECK(q1 == q3);
      BOOST_CHECK(!(q1 == q2));
      BOOST_CHECK(!(q2 == q3));
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Inequality)
    {
      Quaternion q1(1.1f, 2.2f, 3.3f, 4.4f);
      Quaternion q2(4.4f, 6.6f, 7.7f, 8.8f);
      Quaternion q3(1.1f, 2.2f, 3.3f, 4.4f);
      BOOST_CHECK(!(q1 != q3));
      BOOST_CHECK(q1 != q2);
      BOOST_CHECK(q2 != q3);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Index_Operator)
    {
      Quaternion q(0.354f, 0.354f, 0.146f, 0.854f);
      BOOST_CHECK_EQUAL(q[0], 0.354f);
      BOOST_CHECK_EQUAL(q[1], 0.354f);
      BOOST_CHECK_EQUAL(q[2], 0.146f);
      BOOST_CHECK_EQUAL(q[3], 0.854f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Stream_Out)
    {
      Quaternion q(1.1f, 2.2f, 3.3f, 4.4f);
      std::stringstream str;
      str << q;
      BOOST_CHECK_EQUAL("[1.1, 2.2, 3.3, 4.4]", str.str());
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Stream_In)
    {
      std::stringstream str("[1.1, 2.2, 3.3, 4.4]");
      Quaternion q;
      str >> q;
      BOOST_CHECK_EQUAL(q, Quaternion(1.1f, 2.2f, 3.3f, 4.4f));
    }
  }
}
}

#endif
