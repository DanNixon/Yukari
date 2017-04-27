/** @file */

#include "Transform.h"

#include <boost/regex.hpp>

#include <YukariCommon/LoggingService.h>
#include <YukariCommon/StringParsers.h>
#include <YukariMaths/Units.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;

namespace Yukari
{
namespace Maths
{
  Eigen::IOFormat Transform::EIGEN_FORMAT(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",",
                                          "", "", "[", "]");

  Transform::Transform(const Eigen::Matrix4f &mat)
      : m_orientation(Eigen::Quaternionf::Identity())
      , m_position(mat.block(0, 3, 3, 1))
  {
    Eigen::Matrix3f rm = mat.block(0, 0, 3, 3).matrix();
    m_orientation = rm;
  }

  Transform::Transform(const Eigen::Quaternionf &orientation, const Eigen::Vector3f &position)
      : m_orientation(orientation)
      , m_position(position)
  {
  }

  Transform::Transform(const boost::program_options::variables_map &args,
                       const std::string &orientationName, const std::string &positionName)
      : m_orientation(Eigen::Quaternionf::Identity())
      , m_position(Eigen::Vector3f::Zero())
  {
    auto logger = LoggingService::Instance().getLogger("Transform");

    /* Parse orientation */
    if (args.count(orientationName))
    {
      std::string input(args[orientationName].as<std::string>());
      logger->debug("Parsing orientation string \"{}\"", input);

      if (!input.empty())
      {
        Eigen::Vector3f axis;
        float angle;

        boost::regex expression("(\\[.+\\])\\s*([\\-\\.\\w]+)");
        boost::cmatch what;
        if (boost::regex_match(input.c_str(), what, expression))
        {
          axis = StringParsers::ParseVector(what[1]);
          angle = std::stof(what[2]);
        }
        else
        {
          logger->error("Could not parse regex");
        }

        logger->debug("Axis: {}, Angle: {}", axis, angle);
        m_orientation = Eigen::AngleAxisf(angle * DEG_TO_RAD, axis);
      }
      else
      {
        logger->debug("Orientation parameter empty, ignoring");
      }
    }

    /* Parse position */
    if (args.count(positionName))
    {
      std::string input(args[positionName].as<std::string>());
      logger->debug("Parsing position string \"{}\"", input);

      if (!input.empty())
      {
        m_position = StringParsers::ParseVector(input);
      }
      else
      {
        logger->debug("Position parameter empty, ignoring");
      }
    }
  }

  Transform::Transform(const std::string &str)
      : m_orientation(Eigen::Quaternionf::Identity())
      , m_position(Eigen::Vector3f::Zero())
  {
    auto logger = LoggingService::Instance().getLogger("Transform");

    boost::regex expression("\\(\\s*o\\s*\\=\\s*(\\[[\\w\\.\\-\\,\\s]+\\])\\,\\s*p\\s*\\=\\s*(\\[["
                            "\\w\\.\\-\\,\\s]+\\])\\)");

    boost::cmatch what;
    if (boost::regex_match(str.c_str(), what, expression))
    {
      m_orientation = StringParsers::ParseQuaternion(what[1]);
      m_position = StringParsers::ParseVector(what[2]);
    }
    else
    {
      logger->error("Could not parse regex");
    }
  }

  Eigen::Matrix4f Transform::toEigen() const
  {
    Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
    out.block(0, 0, 3, 3) = m_orientation.toRotationMatrix();
    out.block(0, 3, 3, 1) = m_position;
    return out;
  }

  std::ostream &operator<<(std::ostream &s, const Transform &t)
  {
    s << "(o=" << t.m_orientation.coeffs().format(Transform::EIGEN_FORMAT)
      << ", p=" << t.m_position.format(Transform::EIGEN_FORMAT) << ")";

    return s;
  }

  std::istream &operator>>(std::istream &s, Transform &t)
  {
    std::string str;
    std::getline(s, str, ')');
    str += ')';
    t = Transform(str);
    return s;
  }
}
}
