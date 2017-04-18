/** @file */

#pragma once

#include <memory>
#include <vector>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IIMUGrabber.h>
#include <YukariTriggers/ITrigger.h>

#include "IPostCaptureTask.h"
#include "Types.h"

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

    inline void setCloudGrabber(CloudGrabberPtr grabber)
    {
      m_cloudGrabber = grabber;
    }

    inline void setIMUGrabber(IMU::IIMUGrabber_sptr grabber)
    {
      m_imuGrabber = grabber;
    }

    inline void addPostCaptureTask(IPostCaptureTask_sptr task)
    {
      m_postCaptureOperations.push_back(task);
    }

    void addCaptureTrigger(Triggers::ITrigger_sptr trigger);

  private:
    void triggerCapture();

    friend std::ostream &operator<<(std::ostream &s, const CaptureController &o);

  private:
    Common::LoggingService::Logger m_logger;

    bool m_isRunning;
    size_t m_currentFrameCount;

    CloudGrabberPtr m_cloudGrabber;
    IMU::IIMUGrabber_sptr m_imuGrabber;

    std::vector<Triggers::ITrigger_sptr> m_captureTriggers;

    std::vector<IPostCaptureTask_sptr> m_postCaptureOperations;
  };

  typedef std::shared_ptr<CaptureController> CaptureController_sptr;
}
}
