/** @file */

#pragma once

#include "BaseVectorType.h"

namespace Yukari
{
namespace Maths
{
  class Vector4 : public BaseVectorType<4>
  {
  public:
    Vector4(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 1.0f);

    inline float x() const
    {
      return m_values[0];
    }

    inline float &x()
    {
      return m_values[0];
    }

    inline float y() const
    {
      return m_values[1];
    }

    inline float &y()
    {
      return m_values[1];
    }

    inline float z() const
    {
      return m_values[2];
    }

    inline float &z()
    {
      return m_values[2];
    }

    inline float w() const
    {
      return m_values[3];
    }

    inline float &w()
    {
      return m_values[3];
    }

    Vector4 operator+(const Vector4 &rhs) const;
    Vector4 operator-(const Vector4 &rhs) const;
    Vector4 operator*(const Vector4 &rhs) const;
    Vector4 operator/(const Vector4 &rhs) const;

    Vector4 operator*(float rhs) const;
    Vector4 operator/(float rhs) const;
  };
}
}
