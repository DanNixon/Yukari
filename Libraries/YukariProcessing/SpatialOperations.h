/** @file */

#pragma once

#include <Eigen/Geometry>

namespace Yukari
{
namespace Processing
{
  class SpatialOperations
  {
  public:
    static Eigen::Quaternionf RotateQuaternionForCloud(const Eigen::Quaternionf &q)
    {
      Eigen::Quaternionf retVal(q);

      /* Reflect in XZ azis */
      retVal.x() = -retVal.x();
      retVal.z() = -retVal.z();

      return retVal;
    }
  };
}
}
