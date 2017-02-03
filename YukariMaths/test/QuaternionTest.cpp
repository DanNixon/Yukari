/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "QuaternionTest"

#include <boost/test/unit_test.hpp>

#include <YukariMaths/Quaternion.h>

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

    BOOST_AUTO_TEST_CASE(Quaternion_Init_Euler_Degrees)
    {
      Quaternion q(15.0f, 45.0f, 55.0f, DEGREES);
      BOOST_CHECK_EQUAL(q.i(), 0.392f);
      BOOST_CHECK_EQUAL(q.j(), 0.379f);
      BOOST_CHECK_EQUAL(q.k(), 0.282f);
      BOOST_CHECK_EQUAL(q.w(), 0.789f);
    }

    BOOST_AUTO_TEST_CASE(Quaternion_To_Euler)
    {
      Quaternion q(0.392f, 0.397f, 0.282f, 0.789f);
      Vector e = q.toEulerAngles(DEGREES);
      BOOST_CHECK_EQUAL(e.x(), 45.0f);
      BOOST_CHECK_EQUAL(e.y(), 55.0f);
      BOOST_CHECK_EQUAL(e.z(), 15.0);
      BOOST_CHECK_EQUAL(e.w(), 0.0f);
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
      /* BOOST_CHECK_EQUAL(q[0], 0.354f); */
      /* BOOST_CHECK_EQUAL(q[1], 0.354f); */
      /* BOOST_CHECK_EQUAL(q[2], 0.146f); */
      /* BOOST_CHECK_EQUAL(q[3], 0.854f); */
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
