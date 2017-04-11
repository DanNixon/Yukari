/** @file */

#pragma once

#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE>
  class CloudOperations
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;

  public:
	  Cloud::Ptr ApplyTransformationToCloud(Cloud::Ptr cloud, Eigen::Matrix4f transform)
    {
      Cloud::Ptr tc(new ICloudGrabber::Cloud());
      pcl::transformPointCloud(*cloud, *tc, transform);
      return tc;
    }

	  Cloud::Ptr ConcatenateClouds(std::vector<Cloud::Ptr> clouds)
    {
      if (clouds.empty())
        return nullptr;

      if (clouds.size() == 1)
        return clouds.front();

      Cloud::Ptr outputCloud(new ICloudGrabber::Cloud(*(clouds.front())));

      for (auto it = clouds.begin() + 1; it != clouds.end(); ++it)
        *outputCloud += *(*it);

      return outputCloud;
    }

	  Cloud::Ptr RemoveNaNFromCloud(Cloud::Ptr cloud)
    {
      Cloud::Ptr fc(new Cloud());

      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *fc, indices);

      return fc;
    }
  };
}
}
