/** @file */

#pragma once

#include <memory>
#include <vector>

#include <YukariCommon/LoggingService.h>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariIMU/IIMUGrabber.h>

namespace Yukari
{
namespace CaptureApp
{
  class CaptureController
  {
  public:
    CaptureController();

    int run();

  private:
    Common::LoggingService::Logger m_logger;

    CloudCapture::ICloudGrabber_sptr m_cloudGrabber;
    IMU::IIMUGrabber_sptr m_imuGrabber;

    std::vector<ITrigger_sptr> m_captureTriggers;
    std::vector<ITrigger_sptr> m_exitTriggers;
  };

  typedef std::shared_ptr<CaptureController> CaptureController_sptr;
}
}
