/** @file */

#include "Matrix.h"

namespace Yukari
{
namespace Maths
{
  Matrix::Matrix()
  {
    toZero();
  }

  Vector4 Matrix::row(size_t idx) const
  {
    return Vector4(m_values[idx], m_values[idx + 4], m_values[idx + 8], m_values[idx + 12]);
  }

  Vector4 Matrix::column(size_t idx) const
  {
    idx *= 4;
    return Vector4(m_values[idx], m_values[idx + 1], m_values[idx + 2], m_values[idx + 3]);
  }

  void Matrix::setRow(size_t idx, const Vector4 &row)
  {
    m_values[idx] = row.x();
    m_values[idx + 4] = row.y();
    m_values[idx + 8] = row.z();
    m_values[idx + 12] = row.w();
  }

  void Matrix::setColumn(size_t idx, const Vector4 &column)
  {
    idx *= 4;
    m_values[idx] = column.x();
    m_values[idx + 1] = column.y();
    m_values[idx + 2] = column.z();
    m_values[idx + 3] = column.w();
  }

  std::ostream &operator<<(std::ostream &s, const Matrix &m)
  {
    const float *v = m.m_values;

    // clang-format off
    s << '[' << v[0] << ", " << v[4] << ", " << v[8] << ", " << v[12] << ",\n"
      << ' ' << v[1] << ", " << v[5] << ", " << v[9] << ", " << v[13] << ",\n"
      << ' ' << v[2] << ", " << v[6] << ", " << v[10] << ", " << v[14] << ",\n"
      << ' ' << v[3] << ", " << v[7] << ", " << v[11] << ", " << v[15] << ']';
    // clang-format on

    return s;
  }
}
}
