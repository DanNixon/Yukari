/** @file */

#include "ExtendedIMUFrame.h"

namespace Yukari
{
namespace IMU
{
  ExtendedIMUFrame::ExtendedIMUFrame(Duration frameDuration)
      : IMUFrame(frameDuration)
  {
  }

  IMUFrame_sptr ExtendedIMUFrame::predictFrame(float secondsFromNow) const
  {
    /* TODO */
    return std::make_shared<IMUFrame>();
  }

  std::ostream &operator<<(std::ostream &s, const ExtendedIMUFrame &f)
  {
    s << "(dt=" << f.m_durationMs.count() << ", o=" << f.m_orientation << ", p=" << f.m_position
      << ", av=" << f.m_angularVelocity << ", lv=" << f.m_linearVelocity << ")";

    return s;
  }
}
}
