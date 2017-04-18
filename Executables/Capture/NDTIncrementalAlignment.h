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

    inline CloudPtr worldCloud()
    {
      return m_worldCloud;
    }

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) override;

  private:
    Common::LoggingService::Logger m_logger;

    CloudPtr m_worldCloud;
  };
}
}
