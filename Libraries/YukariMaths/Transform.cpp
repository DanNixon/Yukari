/** @file */

#include "Transform.h"

#include <boost/regex.hpp>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Maths
{
  Transform::Transform(const Eigen::Quaternionf &orientation, const Eigen::Vector3f &position)
      : m_orientation(orientation)
      , m_position(position)
  {
  }

  Transform::Transform(const boost::program_options::variables_map &args,
                       const std::string &orientationName, const std::string &positionName)
      : m_orientation()
      , m_position()
  {
    // TODO
    auto logger = Common::LoggingService::Instance().getLogger("Transform");

    /* Parse orientation */
    if (args.count(orientationName))
    {
      std::string input(args[orientationName].as<std::string>());
      logger->debug("Parsing orientation string \"{}\"", input);

      if (!input.empty())
      {
        //Vector3 axis;
        float angle;

        boost::regex expression("(\\[.+\\])\\s*([\\-\\.\\w]+)");
        boost::cmatch what;
        if (boost::regex_match(input.c_str(), what, expression))
        {
          angle = std::stof(what[2]);

          std::stringstream str(what[1]);
          //str >> axis;
        }
        else
        {
          logger->error("Could not parse regex");
        }

        //logger->debug("Axis: {}, Angle: {}", axis, angle);
        //m_orientation = Quaternion(axis, angle, DEGREES);
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
        std::stringstream str(input);
        //str >> m_position;
      }
      else
      {
        logger->debug("Position parameter empty, ignoring");
      }
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
    s << "(o=" << t.m_orientation << ", p=" << t.m_position << ")";

    return s;
  }
}
}
