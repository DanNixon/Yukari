/** @file */

#pragma once

#include <pcl/filters/approximate_voxel_grid.h>

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
    static CloudPtr DownsampleVoxelFilter(CloudConstPtr cloud, double downsamplePercentage)
    {
      auto logger =
          Common::LoggingService::Instance().getLogger("CloudOperations_DownsampleVoxelFilter");
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
