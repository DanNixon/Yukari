/** @file */

#pragma once

#include <chrono>
#include <memory>
#include <ostream>

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

namespace Yukari
{
namespace IMU
{
  class IMUFrame
  {
  public:
    typedef std::chrono::duration<float, std::milli> Duration;

  public:
    IMUFrame(Duration frameDuration = Duration(0.0f),
             const Maths::Quaternion &orientation = Maths::Quaternion(),
             const Maths::Vector3 &position = Maths::Vector3());

    inline Duration duration() const
    {
      return m_durationMs;
    }

    inline Maths::Quaternion orientation() const
    {
      return m_orientation;
    }

    inline Maths::Quaternion &orientation()
    {
      return m_orientation;
    }

    inline Maths::Vector3 position() const
    {
      return m_position;
    }

    inline Maths::Vector3 &position()
    {
      return m_position;
    }

    friend std::ostream &operator<<(std::ostream &s, const IMUFrame &f);

  protected:
    Duration m_durationMs;
    Maths::Quaternion m_orientation;
    Maths::Vector3 m_position;
  };

  typedef std::shared_ptr<IMUFrame> IMUFrame_sptr;
  typedef std::shared_ptr<const IMUFrame> IMUFrame_const_sptr;
}
}
