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

      POINT_TYPE min, max;
      pcl::getMinMax3D<POINT_TYPE>(*cloud, min, max);

      float maxAxis = std::numeric_limits<float>::min();
      float axis;
      for (size_t i = 0; i < 3; i++)
      {
        axis = max.data[i] - min.data[i];
        if (axis > maxAxis)
          maxAxis = axis;
      }

      double leafSize = maxAxis * downsamplePercentage;
      logger->debug("Leaf size: {} (longest axis length: {})", leafSize, maxAxis);

      CloudPtr filteredInputCloud(new Cloud());
      pcl::ApproximateVoxelGrid<POINT_TYPE> voxelFilter;
      voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
      voxelFilter.setInputCloud(cloud);
      voxelFilter.filter(*filteredInputCloud);

      logger->debug("Filtered input cloud size: {} points", filteredInputCloud->size());

      return filteredInputCloud;
    }
  };
}
}
