/** @file */

#include "IMUFrame.h"

#include <Eigen/Geometry>
#include <boost/regex.hpp>

#include <YukariCommon/LoggingService.h>
#include <YukariCommon/StringParsers.h>

using namespace Yukari::Common;
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

  IMUFrame::IMUFrame(const std::string &str)
      : Transform()
  {
    auto logger = LoggingService::Instance().getLogger("IMUFrame");

    boost::regex expression("\\(\\s*dt\\=([\\w\\.\\-]+)\\,\\s*o\\s*\\=\\s*(\\[[\\w\\.\\-\\,\\s]+\\]"
                            ")\\,\\s*p\\s*\\=\\s*(\\[[\\w\\.\\-\\,\\s]+\\])\\)");

    boost::cmatch what;
    if (boost::regex_match(str.c_str(), what, expression))
    {
      m_durationMs = std::chrono::milliseconds(std::stol(what[1]));
      m_orientation = StringParsers::ParseQuaternion(what[2]);
      m_position = StringParsers::ParseVector(what[3]);
    }
    else
    {
      logger->error("Could not parse regex");
    }
  }

  std::ostream &operator<<(std::ostream &s, const IMUFrame &f)
  {
    s << "(dt=" << f.m_durationMs.count()
      << ", o=" << f.m_orientation.coeffs().format(Transform::EIGEN_FORMAT)
      << ", p=" << f.m_position.format(Transform::EIGEN_FORMAT) << ")";

    return s;
  }

  std::istream &operator>>(std::istream &s, IMUFrame &f)
  {
    std::string str;
    std::getline(s, str, ')');
    str += ')';
    f = IMUFrame(str);
    return s;
  }
}
}
