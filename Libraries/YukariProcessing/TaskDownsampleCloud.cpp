/** @file */

#include "TaskDownsampleCloud.h"

#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskDownsampleCloud::TaskDownsampleCloud(const boost::filesystem::path &path,
                                           std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskDownsampleCloud"))
  {
  }

  int TaskDownsampleCloud::process(Task t)
  {
    if (!t.cloud)
    {
      m_logger->error("Do not have a point cloud");
      return 1;
    }

    /* Downsample the input cloud for alignment */
    CloudPtr filteredInputCloud(new Cloud);
    downsample(t.cloud, filteredInputCloud);

    /* Save transformed cloud */
    boost::filesystem::path cloudFilename = formatFilename(t, "_cloud.pcd");
    m_logger->trace("Saving point cloud for frame {}: {}", t.frameNumber, cloudFilename);
    pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *filteredInputCloud);

    return 0;
  }
}
}