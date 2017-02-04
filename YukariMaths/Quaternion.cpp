/** @file */

#include "Quaternion.h"

#include <cmath>

namespace Yukari
{
namespace Maths
{
  Quaternion::Quaternion(float i, float j, float k, float w)
      : BaseMathType()
  {
    m_values[0] = i;
    m_values[1] = j;
    m_values[2] = k;
    m_values[3] = w;
  }

  Quaternion::Quaternion(float roll, float pitch, float yaw, AngleUnit unit)
      : BaseMathType()
  {
    roll *= 0.5f;
    pitch *= 0.5f;
    yaw *= 0.5f;

    if (unit == DEGREES)
    {
      roll *= DEG_TO_RAD;
      pitch *= DEG_TO_RAD;
      yaw *= DEG_TO_RAD;
    }

    const float sinr = sin(roll);
    const float sinp = sin(pitch);
    const float siny = sin(yaw);

    const float cosr = cos(roll);
    const float cosp = cos(pitch);
    const float cosy = cos(yaw);

    m_values[0] = cosr * sinp * cosy + sinr * cosp * siny;
    m_values[1] = cosr * cosp * siny - sinr * sinp * cosy;
    m_values[2] = sinr * cosp * cosy - cosr * sinp * siny;
    m_values[3] = cosr * cosp * cosy + sinr * sinp * siny;
  }

  Vector4 Quaternion::toEulerAngles(AngleUnit unit) const
  {
    Vector4 retVal;

    /* TODO */

    return retVal;
  }

  Vector4 Quaternion::rotate(const Vector4 &v) const
  {
    /* TODO */
    return Vector4();
  }
}
}
