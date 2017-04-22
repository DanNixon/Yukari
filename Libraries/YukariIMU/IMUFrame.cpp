/** @file */

#include "IMUFrame.h"

#include <Eigen/Geometry>

namespace Yukari
{
namespace IMU
{
  IMUFrame::IMUFrame(Duration frameDuration, const Eigen::Quaternionf &orientation,
                     const Eigen::Vector3f &position)
      : Transform(orientation, position)
      , m_durationMs(frameDuration)
  {
  }

  std::ostream &operator<<(std::ostream &s, const IMUFrame &f)
  {
    // TODO
    //s << "(dt=" << f.m_durationMs.count() << ", o=" << f.m_orientation << ", p=" << f.m_position
      //<< ")";

    return s;
  }
}
}
