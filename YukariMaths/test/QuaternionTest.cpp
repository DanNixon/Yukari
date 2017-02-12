/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "QuaternionTest"

#include <boost/math/constants/constants.hpp>
#include <boost/qvm/all.hpp>
#include <boost/test/unit_test.hpp>

#include <YukariMaths/Quaternion.h>

using namespace boost::qvm;

#define TOL 0.001f
#define TOL_ABS 0.00001f

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    /* Boost provided functionality */

    BOOST_AUTO_TEST_CASE(Quaternion_Inverse)
    {
      Quaternion q(5.0, 2.0, 4.5, 8.9);
      Quaternion inv = inverse(q);
      Quaternion r = q * inv;
      BOOST_CHECK_CLOSE(r.w(), 1.0f, TOL);
      BOOST_CHECK_SMALL(r.i(), TOL_ABS);
      BOOST_CHECK_SMALL(r.j(), TOL_ABS);
      BOOST_CHECK_SMALL(r.k(), TOL_ABS);
      BOOST_CHECK_CLOSE(S(r), 1.0f, TOL);
      BOOST_CHECK_SMALL(X(r), TOL_ABS);
      BOOST_CHECK_SMALL(Y(r), TOL_ABS);
      BOOST_CHECK_SMALL(Z(r), TOL_ABS);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Multiply)
    {
      Quaternion q1(1.0f, 0.0f, 1.0f, 0.0f);
      Quaternion q2(1.0f, 0.5f, 0.5f, 0.75f);
      Quaternion q3 = q1 * q2;
      BOOST_CHECK_EQUAL(q3.w(), 0.5f);
      BOOST_CHECK_EQUAL(q3.i(), 1.25f);
      BOOST_CHECK_EQUAL(q3.j(), 1.5f);
      BOOST_CHECK_EQUAL(q3.k(), 0.25f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Multiply_Assignment)
    {
      Quaternion q1(1.0f, 0.0f, 1.0f, 0.0f);
      Quaternion q2(1.0f, 0.5f, 0.5f, 0.75f);
      q1 *= q2;
      BOOST_CHECK_EQUAL(q1.w(), 0.5f);
      BOOST_CHECK_EQUAL(q1.i(), 1.25f);
      BOOST_CHECK_EQUAL(q1.j(), 1.5f);
      BOOST_CHECK_EQUAL(q1.k(), 0.25f);
    }

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

    /* Quaternion functionality */

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Empty)
    {
      Quaternion q;
      BOOST_CHECK_EQUAL(q.w(), 1.0f);
      BOOST_CHECK_EQUAL(q.i(), 0.0f);
      BOOST_CHECK_EQUAL(q.j(), 0.0f);
      BOOST_CHECK_EQUAL(q.k(), 0.0f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Value)
    {
      Quaternion q(0.854f, 0.354f, 0.354f, 0.146f);
      BOOST_CHECK_EQUAL(q.w(), 0.854f);
      BOOST_CHECK_EQUAL(q.i(), 0.354f);
      BOOST_CHECK_EQUAL(q.j(), 0.354f);
      BOOST_CHECK_EQUAL(q.k(), 0.146f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Axis_Angle_Degrees)
    {
      Vector3 axis(1.0f, 1.0f, 1.0f);
      Quaternion q(axis, 90.0f, DEGREES);

      double c = boost::math::constants::one_div_root_two<float>();
      double s = c / std::sqrt(3.0);
      BOOST_CHECK_CLOSE(q[1], s, TOL);
      BOOST_CHECK_CLOSE(q[2], s, TOL);
      BOOST_CHECK_CLOSE(q[3], s, TOL);
      BOOST_CHECK_CLOSE(q[0], c, TOL);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Euler_ZYX_Degrees)
    {
      Quaternion q(Vector3(15.0f, 45.0f, 55.0f), Quaternion::ZYX, DEGREES);
      BOOST_CHECK_CLOSE(q.w(), 0.83554f, 0.01f);
      BOOST_CHECK_CLOSE(q.i(), -0.06822f, 0.01f);
      BOOST_CHECK_CLOSE(q.j(), 0.39222f, 0.01f);
      BOOST_CHECK_CLOSE(q.k(), 0.37864f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Euler_XYZ_Degrees)
    {
      Quaternion q(Vector3(15.0f, 45.0f, 55.0f), Quaternion::XYZ, DEGREES);
      BOOST_CHECK_CLOSE(q.w(), 0.78941f, 0.01f);
      BOOST_CHECK_CLOSE(q.i(), 0.28215f, 0.01f);
      BOOST_CHECK_CLOSE(q.j(), 0.28085f, 0.01f);
      BOOST_CHECK_CLOSE(q.k(), 0.46725f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Euler_YZX_Degrees)
    {
      Quaternion q(Vector3(15.0f, 45.0f, 55.0f), Quaternion::YZX, DEGREES);
      BOOST_CHECK_CLOSE(q.w(), 0.789415f, 0.01f);
      BOOST_CHECK_CLOSE(q.i(), 0.282156f, 0.01f);
      BOOST_CHECK_CLOSE(q.j(), 0.392222f, 0.01f);
      BOOST_CHECK_CLOSE(q.k(), 0.378644f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Rotate_Vector_90Deg_X)
    {
      Quaternion q(Vector3(0.0f, 1.0f, 0.0f), 90.0f, DEGREES);
      Vector3 v(1.0f, 0.0f, 0.0f);
      v = q.rotate(v);
      BOOST_CHECK_CLOSE(v.x(), 0.0f, TOL);
      BOOST_CHECK_CLOSE(v.y(), 0.0f, TOL);
      BOOST_CHECK_CLOSE(v.z(), -1.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_Rotate_Vector_45Deg_Z)
    {
      Quaternion q(Vector3(0.0f, 0.0f, 1.0f), 45.0f, DEGREES);
      Vector3 v(1.0f, 0.0f, 0.0f);
      v = q.rotate(v);
      const double h = sqrt(2.0) / 2;
      BOOST_CHECK_CLOSE(v.x(), h, TOL);
      BOOST_CHECK_CLOSE(v.y(), h, TOL);
      BOOST_CHECK_CLOSE(v.z(), 0.0f, TOL);
    }

    /* BaseMathType functionality */

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
      BOOST_CHECK(q == Quaternion(1.1f, 2.2f, 3.3f, 4.4f));
    }
  }
}
}

#endif
