/** @file */

#include <boost/math/constants/constants.hpp>

namespace Yukari
{
namespace Maths
{
  static const float DEG_TO_RAD = boost::math::constants::pi<float>() / 180.0f;
  static const float RAD_TO_DEG = 180.0f / boost::math::constants::pi<float>();

  enum AngleUnit
  {
    DEGREES,
    RADIANS
  };
}
}
