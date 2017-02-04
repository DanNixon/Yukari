/** @file */

#include "Vector4.h"

namespace Yukari
{
namespace Maths
{
  Vector4::Vector4(float x, float y, float z, float w)
      : BaseVectorType()
  {
    m_values[0] = x;
    m_values[1] = y;
    m_values[2] = z;
    m_values[3] = w;
  }

  Vector4 Vector4::operator+(const Vector4 &rhs) const
  {
    Vector4 v(*this);
    v += rhs;
    return v;
  }

  Vector4 Vector4::operator-(const Vector4 &rhs) const
  {
    Vector4 v(*this);
    v -= rhs;
    return v;
  }

  Vector4 Vector4::operator*(const Vector4 &rhs) const
  {
    Vector4 v(*this);
    v *= rhs;
    return v;
  }

  Vector4 Vector4::operator/(const Vector4 &rhs) const
  {
    Vector4 v(*this);
    v /= rhs;
    return v;
  }

  Vector4 Vector4::operator*(float rhs) const
  {
    Vector4 v(*this);
    v *= rhs;
    return v;
  }

  Vector4 Vector4::operator/(float rhs) const
  {
    Vector4 v(*this);
    v /= rhs;
    return v;
  }
}
}
