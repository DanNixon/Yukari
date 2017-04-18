/** @file */

#include "TaskSaveRawCloud.h"

#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  TaskSaveRawCloud::TaskSaveRawCloud(const boost::filesystem::path &path)
      : m_logger(LoggingService::Instance().getLogger("TaskSaveRawCloud"))
      , m_outputDirectory(path)
  {
    /* Ensure capture directory exists */
    m_logger->info("Capture path: {}", m_outputDirectory);
    boost::filesystem::create_directories(m_outputDirectory);
    m_logger->debug("Capture root directory created");
  }

  int TaskSaveRawCloud::process(size_t frameNumber, CloudConstPtr cloud,
                                IMU::IMUFrame_const_sptr imuFrame)
  {
    if (cloud)
    {
      boost::filesystem::path cloudFilename =
          m_outputDirectory / (std::to_string(frameNumber) + "_cloud.pcd");

      m_logger->trace("Saving point cloud for frame {}: {}", frameNumber, cloudFilename);
      pcl::io::savePCDFileASCII(cloudFilename.string(), *cloud);
    }
    else
    {
      m_logger->error("No point cloud, cannot save");
      return 1;
    }

    return 0;
  }
}
}
