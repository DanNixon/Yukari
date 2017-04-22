/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>

#include "SpatialOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE>
  class TaskAppendTransformedClouds : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    TaskAppendTransformedClouds(const boost::filesystem::path &path)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskAppendTransformedClouds"))
        , m_worldCloud()
    {
    }

    inline CloudPtr worldCloud()
    {
      return m_worldCloud;
    }

    virtual int process(Task t) override
    {
      if (!(t.cloud && t.imuFrame))
      {
        m_logger->error("Do not have both cloud and IMU frame");
        return 1;
      }

      CloudPtr inputCloud(new Cloud());

      /* Transform cloud */
      m_logger->trace("Transforming cloud by IMU");
      pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->position().toEigen(),
                               Processing::SpatialOperations::RotateQuaternionForCloud(
                                   t.imuFrame->orientation().toEigen()));

      if (!m_worldCloud)
      {
        /* If this is the first recored cloud simply set it as he "world" cloud */
        m_worldCloud = CloudPtr(new Cloud(*inputCloud));
      }
      else
      {
        /* Add translated cloud to world cloud */
        *m_worldCloud += *inputCloud;
      }

      return 0;
    }

    virtual int onStop() override
    {
      if (!m_worldCloud)
      {
        m_logger->warn("No world cloud, nothing saved");
        return 1;
      }

      /* Generate filename */
      boost::filesystem::path cloudFilename = m_outputDirectory / "world_cloud.pcd";

      /* Save world cloud */
      m_logger->trace("Saving world point cloud: {}", cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_worldCloud);

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;

    CloudPtr m_worldCloud;
  };
}
}
