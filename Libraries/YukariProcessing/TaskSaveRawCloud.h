/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <boost/filesystem/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>

#include "SpatialOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class TaskSaveRawCloud : public IFrameProcessingTask<POINT_TYPE>
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
        boost::filesystem::path cloudFilename =
            m_outputDirectory / (std::to_string(t.frameNumber) + "_cloud.pcd");

        if (m_transform)
        {
          if (t.imuFrame)
          {
            /* Transform cloud */
            m_logger->trace("Transforming cloud by IMU");
            auto c = new Cloud();
            pcl::transformPointCloud(*t.cloud, *c, t.imuFrame->position().toEigen(),
                                     Processing::SpatialOperations::RotateQuaternionForCloud(
                                         t.imuFrame->orientation().toEigen()));
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
