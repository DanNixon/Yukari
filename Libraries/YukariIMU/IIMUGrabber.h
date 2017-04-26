/** @file */

#pragma once

#include <memory>

#include <YukariMaths/Transform.h>
#include <YukariTriggers/ITrigger.h>

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  class IIMUGrabber
  {
  public:
    typedef std::shared_ptr<IIMUGrabber> Ptr;

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

    virtual Triggers::ITrigger::Ptr trigger()
    {
      return nullptr;
    }

    void setPosition(const Eigen::Vector3f &pos)
    {
      m_transform.position() = pos;
      m_cachedTransform = m_transform.toEigen();
    }

    void setOrientation(const Eigen::Quaternionf &orientation)
    {
      m_transform.orientation() = orientation;
      m_cachedTransform = m_transform.toEigen();
    }

    void setTransform(const Maths::Transform &t)
    {
      m_transform = t;
      m_cachedTransform = m_transform.toEigen();
    }

    virtual IMUFrame::Ptr grabFrame() = 0;

  protected:
    Maths::Transform m_transform;
    Eigen::Matrix4f m_cachedTransform;
  };
}
}
