/** @file */

#include "Quaternion.h"

#include <cmath>

namespace Yukari
{
namespace Maths
{
  Quaternion::Quaternion(float i, float j, float k, float w)
  {
    m_values[0] = i;
    m_values[1] = j;
    m_values[2] = k;
    m_values[3] = w;
  }

  Quaternion::Quaternion(const Vector3 &axis, float angle, AngleUnit unit)
  {
    angle *= 0.5f;

    if (unit == DEGREES)
      angle *= DEG_TO_RAD;

    m_values[3] = std::cos(angle);

    float s = std::sin(angle);
    Vector3 temp(axis);
    temp.normalise();

    m_values[0] = s * temp[0];
    m_values[1] = s * temp[1];
    m_values[2] = s * temp[2];
  }

  Quaternion::Quaternion(float roll, float pitch, float yaw, AngleUnit unit)
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

  float Quaternion::length() const
  {
    return std::sqrt(length2());
  }

  float Quaternion::length2() const
  {
    float len = 0;
    for (size_t i = 0; i < 4; i++)
      len += std::pow(this->m_values[i], 2);
    return len;
  }

  Quaternion Quaternion::conjugate() const
  {
    return Quaternion(-m_values[0], -m_values[1], -m_values[2], m_values[3]);
  }

  Quaternion Quaternion::inverse() const
  {
    Quaternion q = conjugate();

    double m = q.length();
    m *= m;

    if (m == 0.0)
      m = 1.0;
    else
      m = 1.0 / m;

    return Quaternion(q.m_values[0] * m, q.m_values[1] * m, q.m_values[2] * m, q.m_values[3] * m);
  }

  Quaternion Quaternion::operator*(const Quaternion &rhs) const
  {
    Quaternion q(*this);
    q *= rhs;
    return q;
  }

  Quaternion &Quaternion::operator*=(const Quaternion &rhs)
  {
    float il, jl, kl, wl, ir, jr, kr, wr;

    il = m_values[0];
    jl = m_values[1];
    kl = m_values[2];
    wl = m_values[3];

    ir = rhs.m_values[0];
    jr = rhs.m_values[1];
    kr = rhs.m_values[2];
    wr = rhs.m_values[3];

    float i, j, k, w, f;
    f = wl * wr;
    w = f - (il * ir) - (jl * jr) - (kl * kr);
    i = f + (wl * ir) + (jr * kl) - (kr * jl);
    j = f + (wl * jr) + (kr * il) - (ir * kl);
    k = f + (wl * kr) + (ir * jl) - (jr * il);

    m_values[0] = i;
    m_values[1] = j;
    m_values[2] = k;
    m_values[3] = w;

    return (*this);
  }

  Vector3 Quaternion::toEulerAngles(AngleUnit unit) const
  {
    /* TODO */
    return Vector3();
  }

  Vector3 Quaternion::rotate(const Vector3 &v) const
  {
    const Quaternion inv = inverse();
    Quaternion pos(v.x(), v.y(), v.z(), 0.0f);
    pos = pos * inv;
    pos = (*this) * pos;
    return Vector3(pos.i(), pos.j(), pos.k());
  }
}
}
