/** @file */

#include "Quaternion.h"

#include <boost/qvm/all.hpp>
#include <cmath>

using namespace boost::qvm;

namespace Yukari
{
namespace Maths
{
  Quaternion::Quaternion()
  {
    set_identity(*this);
  }

  Quaternion::Quaternion(float i, float j, float k, float w)
  {
    m_values[0] = i;
    m_values[1] = j;
    m_values[2] = k;
    m_values[3] = w;
  }

  Quaternion::Quaternion(const Vector3 &axis, float angle, AngleUnit unit)
  {
    if (unit == DEGREES)
      angle *= DEG_TO_RAD;

    set_rot(*this, axis, angle);
  }

  Quaternion::Quaternion(Vector3 eulerAngles, EulerAngleOrder order, AngleUnit unit)
  {
    if (unit == DEGREES)
      eulerAngles *= DEG_TO_RAD;

    set_identity(*this);

    /* TODO */
    switch (order)
    {
    case ZYX:
      rotate_z(*this, Z(eulerAngles));
      rotate_y(*this, Y(eulerAngles));
      rotate_x(*this, X(eulerAngles));
      break;

    case XYZ:
      rotate_x(*this, X(eulerAngles));
      rotate_y(*this, Y(eulerAngles));
      rotate_z(*this, Z(eulerAngles));
      break;

    case YZX:
      rotate_y(*this, Y(eulerAngles));
      rotate_z(*this, Z(eulerAngles));
      rotate_x(*this, X(eulerAngles));
      break;

    case ZXY:
      rotate_z(*this, Z(eulerAngles));
      rotate_x(*this, X(eulerAngles));
      rotate_y(*this, Y(eulerAngles));
      break;
    }
  }

  float Quaternion::getAngle(AngleUnit unit) const
  {
    float a = 2.0f * std::acos(m_values[0]);
    if (unit == DEGREES)
      a *= RAD_TO_DEG;
    return a;
  }

  Vector3 Quaternion::getAxis() const
  {
    float f = 1.0f / (std::sqrt(1.0f - std::pow(m_values[0], 2)));
    return Vector3(m_values[1] * f, m_values[2] * f, m_values[3] * f);
  }

  Vector3 Quaternion::rotate(const Vector3 &v) const
  {
    const Quaternion inv = inverse(*this);
    Quaternion pos(0.0f, v.x(), v.y(), v.z());
    pos = pos * inv;
    pos = (*this) * pos;
    return Vector3(pos.i(), pos.j(), pos.k());
  }
}
}
