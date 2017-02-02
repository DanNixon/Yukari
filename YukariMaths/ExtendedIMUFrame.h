/** @file */

#pragma once

#include "IMUFrame.h"

namespace Yukari
{
namespace Maths
{
  class ExtendedIMUFrame : public IMUFrame
  {
  public:
    ExtendedIMUFrame();

    inline Vector angularVelocity() const
    {
      return m_angularVelocity;
    }

    inline Vector &angularVelocity()
    {
      return m_angularVelocity;
    }

    inline Vector linearVelocity() const
    {
      return m_linearVelocity;
    }

    inline Vector &linearVelocity()
    {
      return m_linearVelocity;
    }

  protected:
    Vector m_angularVelocity;
    Vector m_linearVelocity;
  };
}
}
