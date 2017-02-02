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

  Vector Matrix::column(size_t idx) const
  {
    return Vector();
  }

  Vector Matrix::row(size_t idx) const
  {
    return Vector();
  }
}
}
