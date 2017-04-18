/** @file */

#include "TaskSaveRawCloud.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <YukariProcessing/SpatialOperations.h>

using namespace Yukari::Common;
using namespace Yukari::Processing;

namespace Yukari
{
namespace CaptureApp
{
  TaskSaveRawCloud::TaskSaveRawCloud(const boost::filesystem::path &path, bool transform)
      : m_logger(LoggingService::Instance().getLogger("TaskSaveRawCloud"))
      , m_outputDirectory(path)
      , m_transform(transform)
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
      /* Generate filename */
      boost::filesystem::path cloudFilename =
          m_outputDirectory / (std::to_string(frameNumber) + "_cloud.pcd");

      if (m_transform)
      {
        /* Transform cloud */
        m_logger->trace("Transforming cloud by IMU");
        auto c = new Cloud();
        pcl::transformPointCloud(
            *cloud, *c, imuFrame->position().toEigen(),
            SpatialOperations::RotateQuaternionForCloud(imuFrame->orientation().toEigen()));
        cloud = CloudConstPtr(c);
      }

      /* Save cloud */
      m_logger->trace("Saving point cloud for frame {}: {}", frameNumber, cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *cloud);
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
