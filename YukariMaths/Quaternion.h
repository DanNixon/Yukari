/** @file */

#pragma once

#include "BaseMathType.h"

#include "Units.h"
#include "Vector.h"

namespace Yukari
{
namespace Maths
{
  class Quaternion : public BaseMathType
  {
  public:
    Quaternion(float i, float j, float k, float w);
    Quaternion(float roll = 0.0f, float pitch = 0.0f, float yaw = 0.0f, AngleUnit unit = RADIANS);

    inline float i() const
    {
      return m_x;
    }

    inline float &i()
    {
      return m_x;
    }

    inline float j() const
    {
      return m_y;
    }

    inline float &j()
    {
      return m_y;
    }

    inline float k() const
    {
      return m_z;
    }

    inline float &k()
    {
      return m_z;
    }

    Vector toEulerAngles(AngleUnit unit = DEGREES) const;

    Vector rotate(const Vector & v) const;
  };
}
}
