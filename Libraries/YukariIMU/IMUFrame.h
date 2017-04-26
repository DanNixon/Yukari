/** @file */

#pragma once

#include <chrono>
#include <iostream>
#include <memory>

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
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity(),
             const Eigen::Vector3f &position = Eigen::Vector3f::Zero());

    IMUFrame(const std::string &str);

    inline Duration duration() const
    {
      return m_durationMs;
    }

    friend std::ostream &operator<<(std::ostream &s, const IMUFrame &f);

  protected:
    Duration m_durationMs;
  };

  std::istream &operator>>(std::istream &s, IMUFrame &f);
}
}
