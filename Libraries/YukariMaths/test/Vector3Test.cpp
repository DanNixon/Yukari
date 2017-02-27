/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "Vector3Test"

#include <boost/qvm/all.hpp>
#include <boost/test/unit_test.hpp>

#include <YukariMaths/Vector3.h>

using namespace boost::qvm;

#define TOL 0.000011f

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    /* Boost provided functionality */

    BOOST_AUTO_TEST_CASE(Vector3_Addition)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(4.4f, 6.6f, 7.7f);
      Vector3 v3 = v1 + v2;
      BOOST_CHECK_CLOSE(v3.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v3.y(), 8.8f, TOL);
      BOOST_CHECK_CLOSE(v3.z(), 11.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Subtraction)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(4.4f, 6.6f, 7.7f);
      Vector3 v3 = v1 - v2;
      BOOST_CHECK_CLOSE(v3.x(), -3.3f, TOL);
      BOOST_CHECK_CLOSE(v3.y(), -4.4f, TOL);
      BOOST_CHECK_CLOSE(v3.z(), -4.4f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Multiplication_By_Scalar)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2 = v1 * 5.0f;
      BOOST_CHECK_CLOSE(v2.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v2.y(), 11.0f, TOL);
      BOOST_CHECK_CLOSE(v2.z(), 16.5f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Division_By_Scalar)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2 = v1 / 5.0f;
      BOOST_CHECK_CLOSE(v2.x(), 0.22f, TOL);
      BOOST_CHECK_CLOSE(v2.y(), 0.44f, TOL);
      BOOST_CHECK_CLOSE(v2.z(), 0.66f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Length)
    {
      Vector3 v(2.0f, 5.0f, 7.0f);
      BOOST_CHECK_CLOSE(mag(v), 8.8317f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Length2)
    {
      Vector3 v(2.0f, 5.0f, 7.0f);
      BOOST_CHECK_CLOSE(mag_sqr(v), 78.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Normalise)
    {
      Vector3 v(2.0f, 5.0f, 7.0f);
      normalize(v);
      BOOST_CHECK_CLOSE(mag(v), 1.0f, TOL);
      BOOST_CHECK_CLOSE(v.x(), 0.22645f, 0.01f);
      BOOST_CHECK_CLOSE(v.y(), 0.5661f, 0.01f);
      BOOST_CHECK_CLOSE(v.z(), 0.7926f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Addition_Assignment)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(4.4f, 6.6f, 7.7f);
      v1 += v2;
      BOOST_CHECK_CLOSE(v1.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 8.8f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 11.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Subtraction_Assignment)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(4.4f, 6.6f, 7.7f);
      v1 -= v2;
      BOOST_CHECK_CLOSE(v1.x(), -3.3f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), -4.4f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), -4.4f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Multiplication_By_Scalar_Assignment)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      v1 *= 5.0f;
      BOOST_CHECK_CLOSE(v1.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 11.0f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 16.5f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Division_By_Scalar_Assignment)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      v1 /= 5.0f;
      BOOST_CHECK_CLOSE(v1.x(), 0.22f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 0.44f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 0.66f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Equality)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(4.4f, 6.6f, 7.7f);
      Vector3 v3(1.1f, 2.2f, 3.3f);
      BOOST_CHECK(v1 == v3);
      BOOST_CHECK(!(v1 == v2));
      BOOST_CHECK(!(v2 == v3));
    }

    BOOST_AUTO_TEST_CASE(Vector3_Inequality)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(4.4f, 6.6f, 7.7f);
      Vector3 v3(1.1f, 2.2f, 3.3f);
      BOOST_CHECK(!(v1 != v3));
      BOOST_CHECK(v1 != v2);
      BOOST_CHECK(v2 != v3);
    }

    /* Vector3 functionality */

    BOOST_AUTO_TEST_CASE(Vector3_Init_Empty)
    {
      Vector3 v;
      BOOST_CHECK_EQUAL(v.x(), 0.0f);
      BOOST_CHECK_EQUAL(v.y(), 0.0f);
      BOOST_CHECK_EQUAL(v.z(), 0.0f);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Init_Value)
    {
      Vector3 v(1.1f, 2.2f, 3.3f);
      BOOST_CHECK_EQUAL(v.x(), 1.1f);
      BOOST_CHECK_EQUAL(v.y(), 2.2f);
      BOOST_CHECK_EQUAL(v.z(), 3.3f);
    }

    BOOST_AUTO_TEST_CASE(Vector3_Init_Copy)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Vector3 v2(v1);
      BOOST_CHECK_EQUAL(v2.x(), 1.1f);
      BOOST_CHECK_EQUAL(v2.y(), 2.2f);
      BOOST_CHECK_EQUAL(v2.z(), 3.3f);
    }

    BOOST_AUTO_TEST_CASE(Vector3_ToEigen)
    {
      Vector3 v1(1.1f, 2.2f, 3.3f);
      Eigen::Vector3f v2 = v1.toEigen();
      BOOST_CHECK_EQUAL(v2.x(), 1.1f);
      BOOST_CHECK_EQUAL(v2.y(), 2.2f);
      BOOST_CHECK_EQUAL(v2.z(), 3.3f);
    }

    /* BaseMathType functionality */

    BOOST_AUTO_TEST_CASE(Vector3_Stream_Out)
    {
      Vector3 v(1.1f, 2.2f, 3.3f);
      std::stringstream str;
      str << v;
      BOOST_CHECK_EQUAL("[1.1, 2.2, 3.3]", str.str());
    }

    BOOST_AUTO_TEST_CASE(Vector3_Stream_In)
    {
      std::stringstream str("[1.1, 2.2, 3.3");
      Vector3 v;
      str >> v;
      BOOST_CHECK(v == Vector3(1.1f, 2.2f, 3.3f));
    }
  }
}
}

#endif
