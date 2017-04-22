/** @file */

#pragma once

#include <memory>
#include <ostream>

#include <Eigen/Geometry>
#include <boost/program_options.hpp>

namespace Yukari
{
namespace Maths
{
  class Transform
  {
  public:
    static Eigen::IOFormat EIGEN_FORMAT;

  public:
    Transform(const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity(),
              const Eigen::Vector3f &position = Eigen::Vector3f::Zero());

    Transform(const boost::program_options::variables_map &args,
              const std::string &orientationName = "orientation",
              const std::string &positionName = "position");

    inline Eigen::Quaternionf orientation() const
    {
      return m_orientation;
    }

    inline Eigen::Quaternionf &orientation()
    {
      return m_orientation;
    }

    inline Eigen::Vector3f position() const
    {
      return m_position;
    }

    inline Eigen::Vector3f &position()
    {
      return m_position;
    }

    Eigen::Matrix4f toEigen() const;

    friend std::ostream &operator<<(std::ostream &s, const Transform &t);

  protected:
    Eigen::Quaternionf m_orientation;
    Eigen::Vector3f m_position;
  };
}
}
