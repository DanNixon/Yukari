/** @file */

#pragma once

#include "BaseMathType.h"

namespace Yukari
{
namespace Maths
{
  class Vector : public BaseMathType
  {
  public:
    Vector(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 1.0f);

    inline float x() const
    {
      return m_x;
    }

    inline float &x()
    {
      return m_x;
    }

    inline float y() const
    {
      return m_y;
    }

    inline float &y()
    {
      return m_y;
    }

    inline float z() const
    {
      return m_z;
    }

    inline float &z()
    {
      return m_z;
    }

    float length() const;
    float length2() const;

    float distance(const Vector &other) const;
    float distance2(const Vector &other) const;

    Vector operator+(const Vector &rhs) const;
    Vector &operator+=(const Vector &rhs);

    Vector operator-(const Vector &rhs) const;
    Vector &operator-=(const Vector &rhs);

    Vector operator*(const Vector &rhs) const;
    Vector &operator*=(const Vector &rhs);
    Vector operator*(float rhs) const;
    Vector &operator*=(float rhs);

    Vector operator/(const Vector &rhs) const;
    Vector &operator/=(const Vector &rhs);
    Vector operator/(float rhs) const;
    Vector &operator/=(float rhs);
  };
}
}
