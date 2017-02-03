/** @file */

#pragma once

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  class ExtendedIMUFrame : public IMUFrame
  {
  public:
    ExtendedIMUFrame();

    inline Maths::Vector angularVelocity() const
    {
      return m_angularVelocity;
    }

    inline Maths::Vector &angularVelocity()
    {
      return m_angularVelocity;
    }

    inline Maths::Vector linearVelocity() const
    {
      return m_linearVelocity;
    }

    inline Maths::Vector &linearVelocity()
    {
      return m_linearVelocity;
    }

    IMUFrame_sptr predictFrame(float secondsFromNow) const;

  protected:
    Maths::Vector m_angularVelocity;
    Maths::Vector m_linearVelocity;
  };

  typedef std::shared_ptr<ExtendedIMUFrame> ExtendedIMUFrame_sptr;
  typedef std::shared_ptr<const ExtendedIMUFrame> ExtendedIMUFrame_const_sptr;
}
}
