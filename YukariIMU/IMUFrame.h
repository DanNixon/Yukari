/** @file */

#pragma once

#include <memory>
#include <chrono>

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

namespace Yukari
{
namespace IMU
{
  class IMUFrame
  {
  public:
    typedef std::chrono::duration<float, std::milli> duration_t;

  public:
    IMUFrame(duration_t frameDuration = duration_t(0.0f));

    inline duration_t duration() const
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

  protected:
    Maths::Quaternion m_orientation;
    Maths::Vector3 m_position;
    duration_t m_durationMs;
  };

  typedef std::shared_ptr<IMUFrame> IMUFrame_sptr;
  typedef std::shared_ptr<const IMUFrame> IMUFrame_const_sptr;
}
}
