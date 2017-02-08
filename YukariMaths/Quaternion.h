/** @file */

#pragma once

#include "BaseMathType.h"

#include <boost/qvm/quat_traits.hpp>

#include "Units.h"
#include "Vector3.h"

namespace Yukari
{
namespace Maths
{
  class Quaternion : public BaseMathType<4>
  {
  public:
    enum EulerAngleOrder
    {
      ZYX,
      XYZ,
      YZX
    };

  public:
    Quaternion();
    Quaternion(float w, float i, float j, float k);
    Quaternion(const Vector3 &axis, float angle, AngleUnit unit = RADIANS);
    Quaternion(Vector3 eulerAngles, EulerAngleOrder order = ZYX, AngleUnit unit = RADIANS);

    inline float i() const
    {
      return m_values[1];
    }

    inline float &i()
    {
      return m_values[1];
    }

    inline float j() const
    {
      return m_values[2];
    }

    inline float &j()
    {
      return m_values[2];
    }

    inline float k() const
    {
      return m_values[3];
    }

    inline float &k()
    {
      return m_values[3];
    }

    inline float w() const
    {
      return m_values[0];
    }

    inline float &w()
    {
      return m_values[0];
    }

    float getAngle(AngleUnit unit = RADIANS) const;
    Vector3 getAxis() const;

    Vector3 rotate(const Vector3 &v) const;

    friend boost::qvm::quat_traits<Quaternion>;
  };
}
}

namespace boost
{
namespace qvm
{
  template <> struct quat_traits<Yukari::Maths::Quaternion>
  {
    typedef float scalar_type;

    template <int I> static inline scalar_type &write_element(Yukari::Maths::Quaternion &q)
    {
      return q.m_values[I];
    }

    template <int I> static inline scalar_type read_element(Yukari::Maths::Quaternion const &q)
    {
      return q.m_values[I];
    }
  };
}
}
