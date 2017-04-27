/** @file */

#pragma once

#include <chrono>
#include <iostream>
#include <memory>

#include <boost/filesystem.hpp>

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
    IMUFrame(const Eigen::Matrix4f &mat, Duration frameDuration = Duration(0.0f));

    IMUFrame(Duration frameDuration = Duration(0.0f),
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity(),
             const Eigen::Vector3f &position = Eigen::Vector3f::Zero());

    IMUFrame(const std::string &str);

    void save(const boost::filesystem::path &path) const;

    inline Duration duration() const
    {
      return m_durationMs;
    }

    Eigen::Matrix4f toCloudTransform() const;

    friend std::ostream &operator<<(std::ostream &s, const IMUFrame &f);

  protected:
    Duration m_durationMs;
  };

  std::istream &operator>>(std::istream &s, IMUFrame &f);
}
}
