/** @file */

#pragma once

#include <memory>
#include <ostream>

#include <Eigen/Geometry>
#include <boost/program_options.hpp>

#include "Quaternion.h"
#include "Vector3.h"

namespace Yukari
{
namespace Maths
{
  class Transform
  {
  public:
    Transform(const Quaternion &orientation = Quaternion(), const Vector3 &position = Vector3());
    Transform(const boost::program_options::variables_map &args,
              const std::string &orientationName = "orientation",
              const std::string &positionName = "position");

    inline Quaternion orientation() const
    {
      return m_orientation;
    }

    inline Quaternion &orientation()
    {
      return m_orientation;
    }

    inline Vector3 position() const
    {
      return m_position;
    }

    inline Vector3 &position()
    {
      return m_position;
    }

    Eigen::Matrix4f toEigen() const;

    friend std::ostream &operator<<(std::ostream &s, const Transform &t);

  protected:
    Quaternion m_orientation;
    Vector3 m_position;
  };
}
}
