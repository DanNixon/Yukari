/** @file */

#pragma once

#include <memory>
#include <vector>

#include <boost/filesystem/path.hpp>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IIMUGrabber.h>
#include <YukariTriggers/ITrigger.h>

#include "Types.h"

namespace Yukari
{
namespace CaptureApp
{
  class CaptureController
  {
  public:
    enum class TransformMode: uint8_t
    {
      SAVE_TRANSFORM,
      TRANSFORM_NOW
    };

  public:
    CaptureController();

    int run();

    bool start();
    bool stop();

    inline bool isRunning() const
    {
      return m_isRunning;
    }

    inline void setOutputDirectory(const boost::filesystem::path &root)
    {
      m_outputDirectory = root;
    }

    inline void setTransformMode(TransformMode mode)
    {
      m_transformMode = mode;
    }

    inline void setCloudGrabber(CloudGrabberPtr grabber)
    {
      m_cloudGrabber = grabber;
    }

    inline void setIMUGrabber(IMU::IIMUGrabber_sptr grabber)
    {
      m_imuGrabber = grabber;
    }

    void addCaptureTrigger(Triggers::ITrigger_sptr trigger);

  private:
    void triggerCapture();

    friend std::ostream &operator<<(std::ostream &s, const CaptureController &o);

  private:
    Common::LoggingService::Logger m_logger;

    bool m_isRunning;
    size_t m_currentFrameCount;

    boost::filesystem::path m_outputDirectory;

    TransformMode m_transformMode;

    CloudGrabberPtr m_cloudGrabber;
    IMU::IIMUGrabber_sptr m_imuGrabber;

    std::vector<Triggers::ITrigger_sptr> m_captureTriggers;
  };

  typedef std::shared_ptr<CaptureController> CaptureController_sptr;
}
}
