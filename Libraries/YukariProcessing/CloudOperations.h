/** @file */

#pragma once

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class CloudOperations
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

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

    static CloudPtr DownsampleVoxelFilter(CloudConstPtr cloud)
    {
      auto logger = Common::LoggingService::Instance().getLogger("CloudOperations_DownsampleVoxelFilter");
      logger->debug("Input cloud size: {} points", cloud->size());

      CloudPtr filteredInputCloud(new Cloud());
      pcl::ApproximateVoxelGrid<POINT_TYPE> voxelFilter;
      voxelFilter.setLeafSize(0.01, 0.01, 0.01); // TODO
      voxelFilter.setInputCloud(cloud);
      voxelFilter.filter(*filteredInputCloud);

      logger->debug("Filtered input cloud size: {} points", filteredInputCloud->size());

      return filteredInputCloud;
    }
  };
}
}
