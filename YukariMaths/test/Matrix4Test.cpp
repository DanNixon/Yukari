/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "Matrix4Test"

#include <boost/qvm/all.hpp>
#include <boost/test/unit_test.hpp>

#include <YukariMaths/Matrix4.h>
#include <YukariMaths/Vector4.h>

using namespace boost::qvm;

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(Matrix4_Set_Row_Get_Column)
    {
      Matrix4 m;
      assign(row<0>(m), Vector4(1.1f, 1.2f, 1.3f, 1.4f));
      assign(row<1>(m), Vector4(2.1f, 2.2f, 2.3f, 2.4f));
      assign(row<2>(m), Vector4(3.1f, 3.2f, 3.3f, 3.4f));
      assign(row<3>(m), Vector4(4.1f, 4.2f, 4.3f, 4.4f));
      BOOST_CHECK(col<0>(m) == Vector4(1.1f, 2.1f, 3.1f, 4.1f));
      BOOST_CHECK(col<1>(m) == Vector4(1.2f, 2.2f, 3.2f, 4.2f));
      BOOST_CHECK(col<2>(m) == Vector4(1.3f, 2.3f, 3.3f, 4.3f));
      BOOST_CHECK(col<3>(m) == Vector4(1.4f, 2.4f, 3.4f, 4.4f));
    }

    BOOST_AUTO_TEST_CASE(Matrix4_Set_Column_Get_Row)
    {
      Matrix4 m;
      assign(col<0>(m), Vector4(1.1f, 2.1f, 3.1f, 4.1f));
      assign(col<1>(m), Vector4(1.2f, 2.2f, 3.2f, 4.2f));
      assign(col<2>(m), Vector4(1.3f, 2.3f, 3.3f, 4.3f));
      assign(col<3>(m), Vector4(1.4f, 2.4f, 3.4f, 4.4f));
      BOOST_CHECK(row<0>(m) == Vector4(1.1f, 1.2f, 1.3f, 1.4f));
      BOOST_CHECK(row<1>(m) == Vector4(2.1f, 2.2f, 2.3f, 2.4f));
      BOOST_CHECK(row<2>(m) == Vector4(3.1f, 3.2f, 3.3f, 3.4f));
      BOOST_CHECK(row<3>(m) == Vector4(4.1f, 4.2f, 4.3f, 4.4f));
    }

    BOOST_AUTO_TEST_CASE(Matrix4_Stream_Out_Row)
    {
      Matrix4 m;
      assign(row<0>(m), Vector4(1.1f, 1.2f, 1.3f, 1.4f));
      assign(row<1>(m), Vector4(2.1f, 2.2f, 2.3f, 2.4f));
      assign(row<2>(m), Vector4(3.1f, 3.2f, 3.3f, 3.4f));
      assign(row<3>(m), Vector4(4.1f, 4.2f, 4.3f, 4.4f));
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL(
          "[1.1, 1.2, 1.3, 1.4,\n 2.1, 2.2, 2.3, 2.4,\n 3.1, 3.2, 3.3, 3.4,\n 4.1, 4.2, 4.3, 4.4]",
          str.str());
    }

    /* Matrix4 functionality */

    BOOST_AUTO_TEST_CASE(Matrix4_Init_Empty)
    {
      Matrix4 m;
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[1, 0, 0, 0,\n 0, 1, 0, 0,\n 0, 0, 1, 0,\n 0, 0, 0, 1]", str.str());
    }

    BOOST_AUTO_TEST_CASE(Matrix4_Stream_Out_Column)
    {
      Matrix4 m;
      assign(col<0>(m), Vector4(1.1f, 2.1f, 3.1f, 4.1f));
      assign(col<1>(m), Vector4(1.2f, 2.2f, 3.2f, 4.2f));
      assign(col<2>(m), Vector4(1.3f, 2.3f, 3.3f, 4.3f));
      assign(col<3>(m), Vector4(1.4f, 2.4f, 3.4f, 4.4f));
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL(
          "[1.1, 1.2, 1.3, 1.4,\n 2.1, 2.2, 2.3, 2.4,\n 3.1, 3.2, 3.3, 3.4,\n 4.1, 4.2, 4.3, 4.4]",
          str.str());
    }
  }
}
}

#endif
