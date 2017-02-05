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
    }
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
