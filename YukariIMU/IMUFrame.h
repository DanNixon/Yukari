/** @file */

#pragma once

#include <memory>

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector.h>

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

    inline Maths::Vector position() const
    {
      return m_position;
    }

    inline Maths::Vector &position()
    {
      return m_position;
    }

  protected:
    Maths::Quaternion m_orientation;
    Maths::Vector m_position;
  };

  typedef std::shared_ptr<IMUFrame> IMUFrame_sptr;
  typedef std::shared_ptr<const IMUFrame> IMUFrame_const_sptr;
}
}
