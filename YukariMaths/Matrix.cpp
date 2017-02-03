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

  bool Matrix::operator==(const Matrix &other) const
  {
    for (size_t i = 0; i < NUM_ELEMENTS; i++)
    {
      if (m_values[i] != other.m_values[i])
        return false;
    }

    return true;
  }

  bool Matrix::operator!=(const Matrix &other) const
  {
    return !this->operator==(other);
  }

  void Matrix::toZero()
  {
    for (size_t i = 0; i < NUM_ELEMENTS; i++)
      m_values[i] = 0.0f;
  }

  Vector Matrix::row(size_t idx) const
  {
    return Vector(m_values[idx], m_values[idx + 4], m_values[idx + 8], m_values[idx + 12]);
  }

  Vector Matrix::column(size_t idx) const
  {
    idx *= 4;
    return Vector(m_values[idx], m_values[idx + 1], m_values[idx + 2], m_values[idx + 3]);
  }

  void Matrix::setRow(size_t idx, const Vector &row)
  {
    m_values[idx] = row.x();
    m_values[idx + 4] = row.y();
    m_values[idx + 8] = row.z();
    m_values[idx + 12] = row.w();
  }

  void Matrix::setColumn(size_t idx, const Vector &column)
  {
    idx *= 4;
    m_values[idx] = column.x();
    m_values[idx + 1] = column.y();
    m_values[idx + 2] = column.z();
    m_values[idx + 3] = column.w();
  }

  std::ostream &operator<<(std::ostream &s, const Matrix &m)
  {
    // clang-format off
    s << '[' << m.m_values[0] << ", " << m.m_values[4] << ", " << m.m_values[8] << ", " << m.m_values[12] << ",\n"
      << ' ' << m.m_values[1] << ", " << m.m_values[5] << ", " << m.m_values[9] << ", " << m.m_values[13] << ",\n"
      << ' ' << m.m_values[2] << ", " << m.m_values[6] << ", " << m.m_values[10] << ", " << m.m_values[14] << ",\n"
      << ' ' << m.m_values[3] << ", " << m.m_values[7] << ", " << m.m_values[11] << ", " << m.m_values[15] << ']';
    // clang-format on

    return s;
  }
}
}
