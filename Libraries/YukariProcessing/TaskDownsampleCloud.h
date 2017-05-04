/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>

#include "CloudOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class TaskDownsampleCloud : public ITaskAlignment<POINT_TYPE>
  {
  public:
    TaskDownsampleCloud(const boost::filesystem::path &path,
                        std::map<std::string, std::string> &params)
        : ITaskAlignment(path, params)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskDownsampleCloud"))
    {
    }

    virtual int process(Task t) override
    {
      if (!t.cloud)
      {
        m_logger->error("Do not have a point cloud");
        return 1;
      }

      /* Format frame number */
      std::stringstream ss;
      ss << std::setw(5) << std::setfill('0') << t.frameNumber;
      std::string frameNoStr = ss.str();

      /* Downsample the input cloud for alignment */
      auto filteredInputCloud = Processing::CloudOperations<POINT_TYPE>::DownsampleVoxelFilter(
          t.cloud, m_voxelDownsamplePercentage);

      /* Save transformed cloud */
      boost::filesystem::path cloudFilename = m_outputDirectory / (frameNoStr + "_cloud.pcd");
      m_logger->trace("Saving point cloud for frame {}: {}", t.frameNumber, cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *filteredInputCloud);

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
