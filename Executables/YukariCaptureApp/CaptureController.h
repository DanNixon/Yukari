/** @file */

#pragma once

#include <memory>
#include <vector>

#include <boost/filesystem/path.hpp>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IIMUGrabber.h>

#include "ITrigger.h"

namespace Yukari
{
namespace CaptureApp
{
  class CaptureController
  {
  public:
    CaptureController();

    int run();

    bool start();
    bool stop();

    inline bool isRunning() const
    {
      return m_isRunning;
    }

    void triggerCapture();

    inline void setCloudGrabber(CloudCapture::ICloudGrabber_sptr grabber)
    {
      m_cloudGrabber = grabber;
    }

    inline void setIMUGrabber(IMU::IIMUGrabber_sptr grabber)
    {
      m_imuGrabber = grabber;
    }

    inline void addCaptureTrigger(ITrigger_sptr trigger)
    {
      m_captureTriggers.push_back(trigger);
    }

    inline void addExitTrigger(ITrigger_sptr trigger)
    {
      m_exitTriggers.push_back(trigger);
    }

  private:
    Common::LoggingService::Logger m_logger;

    bool m_isRunning;

    boost::filesystem::path m_outputRootPath;

    CloudCapture::ICloudGrabber_sptr m_cloudGrabber;
    IMU::IIMUGrabber_sptr m_imuGrabber;

    std::vector<ITrigger_sptr> m_captureTriggers;
    std::vector<ITrigger_sptr> m_exitTriggers;
  };

  typedef std::shared_ptr<CaptureController> CaptureController_sptr;
}
}
