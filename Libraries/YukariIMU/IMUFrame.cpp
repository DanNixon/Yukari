/** @file */

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  IMUFrame::IMUFrame(Duration frameDuration, const Maths::Quaternion &orientation,
                     const Maths::Vector3 &position)
      : Transform(orientation, position)
      , m_durationMs(frameDuration)
  {
  }

  std::ostream &operator<<(std::ostream &s, const IMUFrame &f)
  {
    s << "(dt=" << f.m_durationMs.count() << ", o=" << f.m_orientation << ", p=" << f.m_position
      << ")";

    return s;
  }
}
}
