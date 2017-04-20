/** @file */

#pragma once

#include "IPostCaptureTask.h"

#include <boost/filesystem/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>
#include <YukariProcessing/SpatialOperations.h>

namespace Yukari
{
namespace CaptureApp
{
  class TaskSaveRawCloud : public IPostCaptureTask
  {
  public:
    TaskSaveRawCloud(const boost::filesystem::path &path, bool transform = false)
        : IPostCaptureTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskSaveRawCloud"))
        , m_transform(transform)
    {
    }

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) override
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
          pcl::transformPointCloud(*cloud, *c, imuFrame->position().toEigen(),
                                   Processing::SpatialOperations::RotateQuaternionForCloud(
                                       imuFrame->orientation().toEigen()));
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

  private:
    Common::LoggingService::Logger m_logger;

    bool m_transform;
  };
}
}
