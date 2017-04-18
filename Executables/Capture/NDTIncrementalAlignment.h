/** @file */

#pragma once

#include "IPostCaptureTask.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace CaptureApp
{
  class NDTIncrementalAlignment : public IPostCaptureTask
  {
  public:
    NDTIncrementalAlignment();

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
