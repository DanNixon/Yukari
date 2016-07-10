/** @file */

#include "Matrix4.h"

namespace Yukari
{
namespace Maths
{
  Matrix4::Matrix4()
  {
    toZero();
  }

  Matrix4::~Matrix4()
  {
  }

  bool Matrix4::operator==(const Matrix4 &other) const
  {
    for (size_t i = 0; i < 16; i++)
    {
      if (m_values[i] != other.m_values[i])
        return false;
    }

    return true;
  }

  bool Matrix4::operator!=(const Matrix4 &other) const
  {
    return !this->operator==(other);
  }

  void Matrix4::toZero()
  {
    for (size_t i = 0; i < 16; i++)
      m_values[i] = 0.0f;
  }

  Vector Matrix4::column(size_t idx) const
  {
    return Vector();
  }

  Vector Matrix4::row(size_t idx) const
  {
    return Vector();
  }
}
}
