/** @file */

#pragma once

#include <memory>
#include <vector>

#include <boost/filesystem/path.hpp>

#include <YukariCaptureTriggers/ITrigger.h>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCommon/LoggingService.h>
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

    bool start();
    bool stop();

    inline bool isRunning() const
    {
      return m_isRunning;
    }

    inline void setRootOutputDirectory(const boost::filesystem::path &root)
    {
      m_outputRootPath = root;
    }

    inline void setCloudGrabber(CloudCapture::ICloudGrabber_sptr grabber)
    {
      m_cloudGrabber = grabber;
    }

    inline void setIMUGrabber(IMU::IIMUGrabber_sptr grabber)
    {
      m_imuGrabber = grabber;
    }

    void addStartTrigger(CaptureTriggers::ITrigger_sptr trigger);
    void addStopTrigger(CaptureTriggers::ITrigger_sptr trigger);
    void addCaptureTrigger(CaptureTriggers::ITrigger_sptr trigger);
    void addExitTrigger(CaptureTriggers::ITrigger_sptr trigger);

  private:
    void triggerCapture();

    friend std::ostream &operator<<(std::ostream &s, const CaptureController &o);

  private:
    Common::LoggingService::Logger m_logger;

    bool m_isRunning;
    bool m_shouldStop;
    bool m_shouldExit;
    size_t m_currentFrameCount;

    boost::filesystem::path m_outputRootPath;
    boost::filesystem::path m_currentCaptureRootPath;

    CloudCapture::ICloudGrabber_sptr m_cloudGrabber;
    IMU::IIMUGrabber_sptr m_imuGrabber;

    std::vector<CaptureTriggers::ITrigger_sptr> m_startTriggers;
    std::vector<CaptureTriggers::ITrigger_sptr> m_stopTriggers;
    std::vector<CaptureTriggers::ITrigger_sptr> m_captureTriggers;
    std::vector<CaptureTriggers::ITrigger_sptr> m_exitTriggers;
  };

  typedef std::shared_ptr<CaptureController> CaptureController_sptr;
}
}
