/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "VectorTest"

#include <boost/test/unit_test.hpp>

#include <YukariMaths/Vector.h>

#define TOL 0.000011f

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(Vector_Init_Empty)
    {
      Vector v;
      BOOST_CHECK_EQUAL(v.x(), 0.0f);
      BOOST_CHECK_EQUAL(v.y(), 0.0f);
      BOOST_CHECK_EQUAL(v.z(), 0.0f);
      BOOST_CHECK_EQUAL(v.w(), 1.0f);
    }

    BOOST_AUTO_TEST_CASE(Vector_Init_Value)
    {
      Vector v(1.1f, 2.2f, 3.3f, 4.4f);
      BOOST_CHECK_EQUAL(v.x(), 1.1f);
      BOOST_CHECK_EQUAL(v.y(), 2.2f);
      BOOST_CHECK_EQUAL(v.z(), 3.3f);
      BOOST_CHECK_EQUAL(v.w(), 4.4f);
    }

    BOOST_AUTO_TEST_CASE(Vector_Equality)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      Vector v3(1.1f, 2.2f, 3.3f, 4.4f);
      BOOST_CHECK(v1 == v3);
      BOOST_CHECK(!(v1 == v2));
      BOOST_CHECK(!(v2 == v3));
    }

    BOOST_AUTO_TEST_CASE(Vector_Inequality)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      Vector v3(1.1f, 2.2f, 3.3f, 4.4f);
      BOOST_CHECK(!(v1 != v3));
      BOOST_CHECK(v1 != v2);
      BOOST_CHECK(v2 != v3);
    }

    BOOST_AUTO_TEST_CASE(Vector_Length)
    {
      Vector v(2.0f, 5.0f, 7.0f, 9.0f);
      BOOST_CHECK_CLOSE(v.length(), 12.61f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Vector_Length2)
    {
      Vector v(2.0f, 5.0f, 7.0f, 9.0f);
      BOOST_CHECK_CLOSE(v.length2(), 159.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Distance)
    {
      Vector v1(2.0f, 5.0f, 7.0f, 9.0f);
      Vector v2(4.0f, 8.0f, 11.0f, 14.0f);
      BOOST_CHECK_CLOSE(v1.distance(v2), 7.348f, 0.01f);
    }

    BOOST_AUTO_TEST_CASE(Vector_Distance2)
    {
      Vector v1(2.0f, 5.0f, 7.0f, 9.0f);
      Vector v2(4.0f, 8.0f, 11.0f, 14.0f);
      BOOST_CHECK_CLOSE(v1.distance2(v2), 54.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Addition)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      Vector v3 = v1 + v2;
      BOOST_CHECK_CLOSE(v3.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v3.y(), 8.8f, TOL);
      BOOST_CHECK_CLOSE(v3.z(), 11.0f, TOL);
      BOOST_CHECK_CLOSE(v3.w(), 13.2f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Addition_Assignment)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      v1 += v2;
      BOOST_CHECK_CLOSE(v1.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 8.8f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 11.0f, TOL);
      BOOST_CHECK_CLOSE(v1.w(), 13.2f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Subtraction)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      Vector v3 = v1 - v2;
      BOOST_CHECK_CLOSE(v3.x(), -3.3f, TOL);
      BOOST_CHECK_CLOSE(v3.y(), -4.4f, TOL);
      BOOST_CHECK_CLOSE(v3.z(), -4.4f, TOL);
      BOOST_CHECK_CLOSE(v3.w(), -4.4f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Subtraction_Assignment)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      v1 -= v2;
      BOOST_CHECK_CLOSE(v1.x(), -3.3f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), -4.4f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), -4.4f, TOL);
      BOOST_CHECK_CLOSE(v1.w(), -4.4f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Multiplication_By_Vector)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      Vector v3 = v1 * v2;
      BOOST_CHECK_CLOSE(v3.x(), 4.84f, TOL);
      BOOST_CHECK_CLOSE(v3.y(), 14.52f, TOL);
      BOOST_CHECK_CLOSE(v3.z(), 25.41f, TOL);
      BOOST_CHECK_CLOSE(v3.w(), 38.72f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Multiplication_By_Vector_Assignment)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2(4.4f, 6.6f, 7.7f, 8.8f);
      v1 *= v2;
      BOOST_CHECK_CLOSE(v1.x(), 4.84f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 14.52f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 25.41f, TOL);
      BOOST_CHECK_CLOSE(v1.w(), 38.72f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Multiplication_By_Scalar)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2 = v1 * 5.0f;
      BOOST_CHECK_CLOSE(v2.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v2.y(), 11.0f, TOL);
      BOOST_CHECK_CLOSE(v2.z(), 16.5f, TOL);
      BOOST_CHECK_CLOSE(v2.w(), 22.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Multiplication_By_Scalar_Assignment)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      v1 *= 5.0f;
      BOOST_CHECK_CLOSE(v1.x(), 5.5f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 11.0f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 16.5f, TOL);
      BOOST_CHECK_CLOSE(v1.w(), 22.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Division_By_Vector)
    {
      Vector v1(4.4f, 6.6f, 6.6f, 8.8f);
      Vector v2(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v3 = v1 / v2;
      BOOST_CHECK_CLOSE(v3.x(), 4.0f, TOL);
      BOOST_CHECK_CLOSE(v3.y(), 3.0f, TOL);
      BOOST_CHECK_CLOSE(v3.z(), 2.0f, TOL);
      BOOST_CHECK_CLOSE(v3.w(), 2.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Division_By_Vector_Assignment)
    {
      Vector v1(4.4f, 6.6f, 6.6f, 8.8f);
      Vector v2(1.1f, 2.2f, 3.3f, 4.4f);
      v1 /= v2;
      BOOST_CHECK_CLOSE(v1.x(), 4.0f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 3.0f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 2.0f, TOL);
      BOOST_CHECK_CLOSE(v1.w(), 2.0f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Division_By_Scalar)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      Vector v2 = v1 / 5.0f;
      BOOST_CHECK_CLOSE(v2.x(), 0.22f, TOL);
      BOOST_CHECK_CLOSE(v2.y(), 0.44f, TOL);
      BOOST_CHECK_CLOSE(v2.z(), 0.66f, TOL);
      BOOST_CHECK_CLOSE(v2.w(), 0.88f, TOL);
    }

    BOOST_AUTO_TEST_CASE(Vector_Division_By_Scalar_Assignment)
    {
      Vector v1(1.1f, 2.2f, 3.3f, 4.4f);
      v1 /= 5.0f;
      BOOST_CHECK_CLOSE(v1.x(), 0.22f, TOL);
      BOOST_CHECK_CLOSE(v1.y(), 0.44f, TOL);
      BOOST_CHECK_CLOSE(v1.z(), 0.66f, TOL);
      BOOST_CHECK_CLOSE(v1.w(), 0.88f, TOL);
    }
  }
}
}

#endif
