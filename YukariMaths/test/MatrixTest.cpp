/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "MatrixTest"

#include <boost/test/unit_test.hpp>

#include <YukariMaths/Matrix.h>

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(Matrix_Init_Empty)
    {
      Matrix m;
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[0, 0, 0, 0,\n 0, 0, 0, 0,\n 0, 0, 0, 0,\n 0, 0, 0, 0]", str.str());
    }

    BOOST_AUTO_TEST_CASE(Matrix_Set_Row_Get_Column)
    {
      Matrix m;
      m.setRow(0, Vector4(1.1f, 1.2f, 1.3f, 1.4f));
      m.setRow(1, Vector4(2.1f, 2.2f, 2.3f, 2.4f));
      m.setRow(2, Vector4(3.1f, 3.2f, 3.3f, 3.4f));
      m.setRow(3, Vector4(4.1f, 4.2f, 4.3f, 4.4f));
      BOOST_CHECK_EQUAL(m.column(0), Vector4(1.1f, 2.1f, 3.1f, 4.1f));
      BOOST_CHECK_EQUAL(m.column(1), Vector4(1.2f, 2.2f, 3.2f, 4.2f));
      BOOST_CHECK_EQUAL(m.column(2), Vector4(1.3f, 2.3f, 3.3f, 4.3f));
      BOOST_CHECK_EQUAL(m.column(3), Vector4(1.4f, 2.4f, 3.4f, 4.4f));
    }

    BOOST_AUTO_TEST_CASE(Matrix_Set_Column_Get_Row)
    {
      Matrix m;
      m.setColumn(0, Vector4(1.1f, 2.1f, 3.1f, 4.1f));
      m.setColumn(1, Vector4(1.2f, 2.2f, 3.2f, 4.2f));
      m.setColumn(2, Vector4(1.3f, 2.3f, 3.3f, 4.3f));
      m.setColumn(3, Vector4(1.4f, 2.4f, 3.4f, 4.4f));
      BOOST_CHECK_EQUAL(m.row(0), Vector4(1.1f, 1.2f, 1.3f, 1.4f));
      BOOST_CHECK_EQUAL(m.row(1), Vector4(2.1f, 2.2f, 2.3f, 2.4f));
      BOOST_CHECK_EQUAL(m.row(2), Vector4(3.1f, 3.2f, 3.3f, 3.4f));
      BOOST_CHECK_EQUAL(m.row(3), Vector4(4.1f, 4.2f, 4.3f, 4.4f));
    }

    BOOST_AUTO_TEST_CASE(Matrix_Stream_Out_Row)
    {
      Matrix m;
      m.setRow(0, Vector4(1.1f, 1.2f, 1.3f, 1.4f));
      m.setRow(1, Vector4(2.1f, 2.2f, 2.3f, 2.4f));
      m.setRow(2, Vector4(3.1f, 3.2f, 3.3f, 3.4f));
      m.setRow(3, Vector4(4.1f, 4.2f, 4.3f, 4.4f));
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[1.1, 1.2, 1.3, 1.4,\n 2.1, 2.2, 2.3, 2.4,\n 3.1, 3.2, 3.3, 3.4,\n 4.1, 4.2, 4.3, 4.4]", str.str());
    }

    BOOST_AUTO_TEST_CASE(Matrix_Stream_Out_Column)
    {
      Matrix m;
      m.setColumn(0, Vector4(1.1f, 2.1f, 3.1f, 4.1f));
      m.setColumn(1, Vector4(1.2f, 2.2f, 3.2f, 4.2f));
      m.setColumn(2, Vector4(1.3f, 2.3f, 3.3f, 4.3f));
      m.setColumn(3, Vector4(1.4f, 2.4f, 3.4f, 4.4f));
      std::stringstream str;
      str << m;
      BOOST_CHECK_EQUAL("[1.1, 1.2, 1.3, 1.4,\n 2.1, 2.2, 2.3, 2.4,\n 3.1, 3.2, 3.3, 3.4,\n 4.1, 4.2, 4.3, 4.4]", str.str());
    }
  }
}
}

#endif
