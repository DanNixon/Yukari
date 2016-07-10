/** @file */

#include "Vector.h"

#include <cmath>

namespace Yukari
{
namespace Maths
{
  Vector::Vector(float x, float y, float z, float w)
      : BaseMathType(x, y, z, w)
  {
  }

  Vector::~Vector()
  {
  }

  float Vector::length() const
  {
    return std::sqrt(length2());
  }

  float Vector::length2() const
  {
    return (m_x * m_x) + (m_y * m_y) + (m_z * m_z) + (m_w * m_w);
  }

  float Vector::distance(const Vector &other) const
  {
    return std::sqrt(distance2(other));
  }

  float Vector::distance2(const Vector &other) const
  {
    Vector delta = (*this) - other;
    return delta.length2();
  }

  Vector Vector::operator+(const Vector &rhs) const
  {
    return Vector(m_x + rhs.m_x, m_y + rhs.m_y, m_z + rhs.m_z, m_w + rhs.m_w);
  }

  Vector &Vector::operator+=(const Vector &rhs)
  {
    m_x += rhs.m_x;
    m_y += rhs.m_y;
    m_z += rhs.m_z;
    m_w += rhs.m_w;
    return *this;
  }

  Vector Vector::operator-(const Vector &rhs) const
  {
    return Vector(m_x - rhs.m_x, m_y - rhs.m_y, m_z - rhs.m_z, m_w - rhs.m_w);
  }

  Vector &Vector::operator-=(const Vector &rhs)
  {
    m_x -= rhs.m_x;
    m_y -= rhs.m_y;
    m_z -= rhs.m_z;
    m_w -= rhs.m_w;
    return *this;
  }

  Vector Vector::operator*(const Vector &rhs) const
  {
    return Vector(m_x * rhs.m_x, m_y * rhs.m_y, m_z * rhs.m_z, m_w * rhs.m_w);
  }

  Vector &Vector::operator*=(const Vector &rhs)
  {
    m_x *= rhs.m_x;
    m_y *= rhs.m_y;
    m_z *= rhs.m_z;
    m_w *= rhs.m_w;
    return *this;
  }

  Vector Vector::operator*(float rhs) const
  {
    return Vector(m_x * rhs, m_y * rhs, m_z * rhs, m_w * rhs);
  }

  Vector &Vector::operator*=(float rhs)
  {
    m_x *= rhs;
    m_y *= rhs;
    m_z *= rhs;
    m_w *= rhs;
    return *this;
  }

  Vector Vector::operator/(const Vector &rhs) const
  {
    return Vector(m_x / rhs.m_x, m_y / rhs.m_y, m_z / rhs.m_z, m_w / rhs.m_w);
  }

  Vector &Vector::operator/=(const Vector &rhs)
  {
    m_x /= rhs.m_x;
    m_y /= rhs.m_y;
    m_z /= rhs.m_z;
    m_w /= rhs.m_w;
    return *this;
  }

  Vector Vector::operator/(float rhs) const
  {
    return Vector(m_x / rhs, m_y / rhs, m_z / rhs, m_w / rhs);
  }

  Vector &Vector::operator/=(float rhs)
  {
    m_x /= rhs;
    m_y /= rhs;
    m_z /= rhs;
    m_w /= rhs;
    return *this;
  }
}
}
