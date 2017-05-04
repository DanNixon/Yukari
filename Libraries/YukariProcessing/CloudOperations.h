/** @file */

#pragma once

#include <pcl/common/common.h>
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
    /**
     * @brief Downsamples a point cloud using a voxel grid.
     * @param cloud Input cloud
     * @param downsamplePercentage Width of the voxel grid as a percentage of the eidth of the
     *                             largest axis in the input cloud
     * @return Downsampled cloud
     */
    static CloudPtr DownsampleVoxelFilter(CloudConstPtr cloud, double downsamplePercentage)
    {
      auto logger =
          Common::LoggingService::Instance().getLogger("CloudOperations_DownsampleVoxelFilter");
      logger->debug("Input cloud size: {} points", cloud->size());

      CloudPtr filteredInputCloud(new Cloud());
      pcl::ApproximateVoxelGrid<POINT_TYPE> voxelFilter;
      voxelFilter.setLeafSize(downsamplePercentage, downsamplePercentage, downsamplePercentage);
      voxelFilter.setInputCloud(cloud);
      voxelFilter.filter(*filteredInputCloud);

      logger->debug("Filtered input cloud size: {} points", filteredInputCloud->size());

      return filteredInputCloud;
    }
  };
}
}
