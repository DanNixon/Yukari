/** @file */

#include "Quaternion.h"

#include <cmath>

namespace Yukari
{
namespace Maths
{
  Quaternion::Quaternion(float i, float j, float k, float w)
      : BaseMathType(i, j, k, w)
  {
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

    m_x = cosr * sinp * cosy + sinr * cosp * siny;
    m_y = cosr * cosp * siny - sinr * sinp * cosy;
    m_z = sinr * cosp * cosy - cosr * sinp * siny;
    m_w = cosr * cosp * cosy + sinr * sinp * siny;
  }

  Vector Quaternion::toEulerAngles(AngleUnit unit) const
  {
    Vector retVal;

    float ysqr = m_y * m_y;

    // roll (x-axis rotation)
    float t0 = 2.0 * (m_w * m_x + m_y * m_z);
    float t1 = 1.0 - 2.0 * (m_x * m_x + ysqr);
    retVal.x() = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = 2.0 * (m_w * m_y - m_z * m_x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    retVal.y() = std::asin(t2);

    // yaw (z-axis rotation)
    float t3 = 2.0 * (m_w * m_z + m_x * m_y);
    float t4 = 1.0 - 2.0 * (ysqr + m_z * m_z);
    retVal.z() = std::atan2(t3, t4);

    if (unit == DEGREES)
      retVal *= RAD_TO_DEG;

    retVal.w() = 0.0f;

    return retVal;
  }

  Vector Quaternion::rotate(const Vector &v) const
  {
    /* TODO */
    return Vector();
  }
}
}
