/** @file */

#pragma once

#include <pcl/point_representation.h>

namespace Yukari
{
namespace Processing
{
  class PairRegistrationPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
  {
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

  public:
    PairRegistrationPointRepresentation()
    {
      nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.curvature;
    }
  };
}
}