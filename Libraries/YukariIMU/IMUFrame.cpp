/** @file */

#include "IMUFrame.h"

#include <Eigen/Geometry>

using namespace Yukari::Maths;

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
    s << "(dt=" << f.m_durationMs.count()
      << ", o=" << f.m_orientation.coeffs().format(Transform::EIGEN_FORMAT)
      << ", p=" << f.m_position.format(Transform::EIGEN_FORMAT) << ")";

    return s;
  }
}
}
