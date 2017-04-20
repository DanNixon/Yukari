/** @file */

#pragma once

#include <chrono>
#include <memory>
#include <ostream>

#include <YukariMaths/Transform.h>

namespace Yukari
{
namespace IMU
{
  class IMUFrame : public Maths::Transform
  {
  public:
    typedef std::chrono::duration<float, std::milli> Duration;

    typedef std::shared_ptr<IMUFrame> Ptr;
    typedef std::shared_ptr<const IMUFrame> ConstPtr;

  public:
    IMUFrame(Duration frameDuration = Duration(0.0f),
             const Maths::Quaternion &orientation = Maths::Quaternion(),
             const Maths::Vector3 &position = Maths::Vector3());

    inline Duration duration() const
    {
      return m_durationMs;
    }

    friend std::ostream &operator<<(std::ostream &s, const IMUFrame &f);

  protected:
    Duration m_durationMs;
  };
}
}
