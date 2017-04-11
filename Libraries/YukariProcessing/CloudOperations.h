/** @file */

#pragma once

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class CloudOperations
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

  public:
    static CloudPtr ApplyTransformationToCloud(CloudPtr cloud, Eigen::Matrix4f transform)
    {
      CloudPtr tc(new Cloud());
      pcl::transformPointCloud(*cloud, *tc, transform);
      return tc;
    }

    static CloudPtr ConcatenateClouds(std::vector<CloudPtr> clouds)
    {
      if (clouds.empty())
        return nullptr;

      if (clouds.size() == 1)
        return clouds.front();

      CloudPtr outputCloud(new Cloud(*(clouds.front())));

      for (auto it = clouds.begin() + 1; it != clouds.end(); ++it)
        *outputCloud += *(*it);

      return outputCloud;
    }

    static CloudPtr RemoveNaNFromCloud(CloudPtr cloud)
    {
      CloudPtr fc(new Cloud());

      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *fc, indices);

      return fc;
    }
  };
}
}
