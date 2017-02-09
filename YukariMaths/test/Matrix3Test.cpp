/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "Matrix3Test"

#include <boost/qvm/all.hpp>
#include <boost/test/unit_test.hpp>

#include <YukariMaths/Matrix3.h>
#include <YukariMaths/Vector3.h>

using namespace boost::qvm;

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(Matrix3_Set_Row_Get_Column)
    {
      Matrix3 m;
      assign(row<0>(m), Vector3(1.1f, 1.2f, 1.3f));
      assign(row<1>(m), Vector3(2.1f, 2.2f, 2.3f));
      assign(row<2>(m), Vector3(3.1f, 3.2f, 3.3f));
      BOOST_CHECK(col<0>(m) == Vector3(1.1f, 2.1f, 3.1f));
      BOOST_CHECK(col<1>(m) == Vector3(1.2f, 2.2f, 3.2f));
      BOOST_CHECK(col<2>(m) == Vector3(1.3f, 2.3f, 3.3f));
    }

    BOOST_AUTO_TEST_CASE(Matrix3_Set_Column_Get_Row)
    {
      Matrix3 m;
      assign(col<0>(m), Vector3(1.1f, 2.1f, 3.1f));
      assign(col<1>(m), Vector3(1.2f, 2.2f, 3.2f));
      assign(col<2>(m), Vector3(1.3f, 2.3f, 3.3f));
      BOOST_CHECK(row<0>(m) == Vector3(1.1f, 1.2f, 1.3f));
      BOOST_CHECK(row<1>(m) == Vector3(2.1f, 2.2f, 2.3f));
      BOOST_CHECK(row<2>(m) == Vector3(3.1f, 3.2f, 3.3f));
    }

    BOOST_AUTO_TEST_CASE(Matrix3_Stream_Out_Row)
    {
      Matrix3 m;
      assign(row<0>(m), Vector3(1.1f, 1.2f, 1.3f));
      assign(row<1>(m), Vector3(2.1f, 2.2f, 2.3f));
      assign(row<2>(m), Vector3(3.1f, 3.2f, 3.3f));
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[1.1, 1.2, 1.3,\n 2.1, 2.2, 2.3,\n 3.1, 3.2, 3.3]", str.str());
    }

    /* Matrix3 functionality */

    BOOST_AUTO_TEST_CASE(Matrix3_Init_Empty)
    {
      Matrix3 m;
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[1, 0, 0,\n 0, 1, 0,\n 0, 0, 1]", str.str());
    }

    BOOST_AUTO_TEST_CASE(Matrix3_Stream_Out_Column)
    {
      Matrix3 m;
      assign(col<0>(m), Vector3(1.1f, 2.1f, 3.1f));
      assign(col<1>(m), Vector3(1.2f, 2.2f, 3.2f));
      assign(col<2>(m), Vector3(1.3f, 2.3f, 3.3f));
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[1.1, 1.2, 1.3,\n 2.1, 2.2, 2.3,\n 3.1, 3.2, 3.3]", str.str());
    }
  }
}
}

#endif
