/** @file */

#pragma once

#include <memory>

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

namespace Yukari
{
namespace IMU
{
  class IMUFrame
  {
  public:
    IMUFrame();

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
  };

  typedef std::shared_ptr<IMUFrame> IMUFrame_sptr;
  typedef std::shared_ptr<const IMUFrame> IMUFrame_const_sptr;
}
}
