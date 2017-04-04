/** @file */

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  IMUFrame::IMUFrame(Duration frameDuration, const Maths::Quaternion &orientation,
                     const Maths::Vector3 &position)
      : m_durationMs(frameDuration)
      , m_orientation(orientation)
      , m_position(position)
  {
  }

  Eigen::Matrix4f IMUFrame::toEigen() const
  {
    Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
    out.block(0, 0, 3, 3) = m_orientation.toEigen().toRotationMatrix();
    out.block(0, 3, 3, 1) = m_position.toEigen();
    return out;
  }

  std::ostream &operator<<(std::ostream &s, const IMUFrame &f)
  {
    s << "(dt=" << f.m_durationMs.count() << ", o=" << f.m_orientation << ", p=" << f.m_position
      << ")";

    return s;
  }
}
}
