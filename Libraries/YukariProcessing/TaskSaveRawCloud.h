/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <iomanip>
#include <sstream>

#include <boost/filesystem/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskSaveRawCloud : public IFrameProcessingTask
  {
  public:
    TaskSaveRawCloud(const boost::filesystem::path &path, bool transform = false)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskSaveRawCloud"))
        , m_transform(transform)
    {
    }

    virtual int process(Task t) override
    {
      if (t.cloud)
      {
        /* Generate filename */
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << t.frameNumber;
        boost::filesystem::path cloudFilename = m_outputDirectory / (ss.str() + "_cloud.pcd");

        if (m_transform)
        {
          if (t.imuFrame)
          {
            /* Transform cloud */
            m_logger->trace("Transforming cloud by IMU");
            auto c = new Cloud();
            pcl::transformPointCloud(*t.cloud, *c, t.imuFrame->toCloudTransform());
            t.cloud = CloudConstPtr(c);
          }
          else
          {
            m_logger->warn("Requested transform but no IMU frame");
          }
        }

        /* Save cloud */
        m_logger->trace("Saving point cloud for frame {}: {}", t.frameNumber, cloudFilename);
        pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *t.cloud);
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
