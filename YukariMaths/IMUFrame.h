/** @file */

#pragma once

#include "Vector.h"
#include "Quaternion.h"

namespace Yukari
{
namespace Maths
{
  class IMUFrame
  {
  public:
    IMUFrame();

    inline Quaternion orientation() const
    {
      return m_orientation;
    }

    inline Quaternion &orientation()
    {
      return m_orientation;
    }

    inline Vector position() const
    {
      return m_position;
    }

    inline Vector &position()
    {
      return m_position;
    }

  protected:
    Quaternion m_orientation;
    Vector m_position;
  };
}
}
