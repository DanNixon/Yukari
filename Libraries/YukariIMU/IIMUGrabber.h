/** @file */

#pragma once

#include <memory>

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  class IIMUGrabber
  {
  public:
    virtual void open()
    {
    }

    virtual void close()
    {
    }

    virtual bool isOpen() const
    {
      return true;
    }

    void setPosition(const Maths::Vector3 &pos)
    {
      m_transform.position() = pos;
      m_cachedTransform = m_transform.toEigen();
    }

    void setOrientation(const Maths::Quaternion &orientation)
    {
      m_transform.orientation() = orientation;
      m_cachedTransform = m_transform.toEigen();
    }

    virtual IMUFrame_sptr grabFrame() = 0;

  protected:
    IMUFrame m_transform;
    Eigen::Matrix4f m_cachedTransform;
  };

  typedef std::shared_ptr<IIMUGrabber> IIMUGrabber_sptr;
}
}
