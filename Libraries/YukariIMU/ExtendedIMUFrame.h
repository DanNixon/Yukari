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
    ExtendedIMUFrame(Duration frameDuration = Duration(0.0f));

    inline Maths::Vector3 angularVelocity() const
    {
      return m_angularVelocity;
    }

    inline Maths::Vector3 &angularVelocity()
    {
      return m_angularVelocity;
    }

    inline Maths::Vector3 linearVelocity() const
    {
      return m_linearVelocity;
    }

    inline Maths::Vector3 &linearVelocity()
    {
      return m_linearVelocity;
    }

    IMUFrame_sptr predictFrame(float secondsFromNow) const;

    friend std::ostream &operator<<(std::ostream &s, const ExtendedIMUFrame &f);

  protected:
    Maths::Vector3 m_angularVelocity;
    Maths::Vector3 m_linearVelocity;
  };

  typedef std::shared_ptr<ExtendedIMUFrame> ExtendedIMUFrame_sptr;
  typedef std::shared_ptr<const ExtendedIMUFrame> ExtendedIMUFrame_const_sptr;
}
}
