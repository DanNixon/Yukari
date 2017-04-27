/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>

#include "CloudOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE>
  class TaskNDTWorldSegmentAlignment : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    TaskNDTWorldSegmentAlignment(const boost::filesystem::path &path)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskNDTWorldSegmentAlignment"))
    {
    }

    virtual int process(Task t) override
    {
      if (!(t.cloud && t.imuFrame))
      {
        m_logger->error("Do not have both cloud and IMU frame");
        return 1;
      }

      /* TODO */

      return 0;
    }

    virtual int onStop() override
    {
      /* TODO */

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
