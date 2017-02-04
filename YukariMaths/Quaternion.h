/** @file */

#pragma once

#include "BaseMathType.h"

#include "Units.h"
#include "Vector4.h"

namespace Yukari
{
namespace Maths
{
  class Quaternion : public BaseMathType<4>
  {
  public:
    Quaternion(float i, float j, float k, float w);
    Quaternion(float roll = 0.0f, float pitch = 0.0f, float yaw = 0.0f, AngleUnit unit = RADIANS);

    inline float i() const
    {
      return m_values[0];
    }

    inline float &i()
    {
      return m_values[0];
    }

    inline float j() const
    {
      return m_values[1];
    }

    inline float &j()
    {
      return m_values[1];
    }

    inline float k() const
    {
      return m_values[2];
    }

    inline float &k()
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

    Vector4 toEulerAngles(AngleUnit unit = DEGREES) const;

    Vector4 rotate(const Vector4 & v) const;
  };
}
}
