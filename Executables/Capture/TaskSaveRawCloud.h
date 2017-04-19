/** @file */

#pragma once

#include "IPostCaptureTask.h"

#include <boost/filesystem/path.hpp>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace CaptureApp
{
  class TaskSaveRawCloud : public IPostCaptureTask
  {
  public:
    TaskSaveRawCloud(const boost::filesystem::path &path, bool transform = false);

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) override;

  private:
    Common::LoggingService::Logger m_logger;

    bool m_transform;
  };
}
}
