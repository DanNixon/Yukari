/** @file */

#pragma once

#include "BaseMathType.h"

#include "Units.h"
#include "Vector3.h"

namespace Yukari
{
namespace Maths
{
  class Quaternion : public BaseMathType<4>
  {
  public:
    Quaternion(float i, float j, float k, float w);
    Quaternion(const Vector3 &axis, float angle, AngleUnit unit = RADIANS);
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

    float length() const;
    float length2() const;

    Quaternion conjugate() const;
    Quaternion inverse() const;

    Quaternion operator*(const Quaternion &rhs) const;
    Quaternion &operator*=(const Quaternion &rhs);

    Vector3 toEulerAngles(AngleUnit unit = DEGREES) const;

    Vector3 rotate(const Vector3 &v) const;
  };
}
}
