/** @file */

#include "CaptureController.h"

#include <typeinfo>

#include <YukariIMU/IMUFrame.h>

using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Triggers;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController::CaptureController()
      : m_logger(LoggingService::Instance().getLogger("CaptureController"))
      , m_isRunning(false)
  {
  }

  int CaptureController::run()
  {
    start();

    /* Wait for termination signal */
    while (m_isRunning)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /* Ensure capture is stopped before exit */
    stop();

    /* Ensure log is flushed before exiting */
    LoggingService::Instance().flush();

    return 0;
  }

  bool CaptureController::start()
  {
    if (m_isRunning)
    {
      m_logger->error("Already running");
      return false;
    }

    /* Open IMU grabber */
    m_logger->info("Opening IMU grabber");
    if (m_imuGrabber)
    {
      m_imuGrabber->open();
      if (!m_imuGrabber->isOpen())
      {
        m_logger->error("Failed to open IMU grabber!");
        return false;
      }
      m_logger->debug("IMU grabber opened");
    }
    else
    {
      m_logger->info("No IMU grabber defined");
    }

    /* Open cloud grabber */
    m_logger->info("Opening cloud grabber");
    m_cloudGrabber->open();
    if (!m_cloudGrabber->isOpen())
    {
      m_logger->error("Failed to open cloud grabber!");
      return false;
    }
    m_logger->debug("Cloud grabber opened");

    /* Reset frrame count */
    m_currentFrameCount = 0;

    /* Enable capture triggers */
    m_logger->info("Enabling capture triggers");
    for (auto it = m_captureTriggers.begin(); it != m_captureTriggers.end(); ++it)
      (*it)->enable();

    /* Set running flags */
    m_isRunning = true;

    return true;
  }

  bool CaptureController::stop()
  {
    if (!m_isRunning)
    {
      m_logger->error("Already stopped");
      return false;
    }

    /* Clear running flag */
    m_isRunning = false;

    /* Disable capture triggers */
    m_logger->info("Disabling capture triggers");
    for (auto it = m_captureTriggers.begin(); it != m_captureTriggers.end(); ++it)
      (*it)->disable();

    /* Close IMU grabber */
    m_logger->info("Closing IMU grabber");
    if (m_imuGrabber)
    {
      m_imuGrabber->close();
      if (m_imuGrabber->isOpen())
      {
        m_logger->error("Failed to close IMU grabber!");
        return false;
      }
    }
    else
    {
      m_logger->debug("No IMU grabber defined");
    }

    /* Close cloud grabber */
    m_logger->info("Closing cloud grabber");
    m_cloudGrabber->close();
    if (m_cloudGrabber->isOpen())
    {
      m_logger->error("Failed to close cloud grabber!");
      return false;
    }

    return true;
  }

  void CaptureController::addCaptureTrigger(ITrigger_sptr trigger)
  {
    trigger->setHandler([this]() { triggerCapture(); });
    m_captureTriggers.push_back(trigger);
    m_logger->debug("Added capture trigger");
  }

  void CaptureController::triggerCapture()
  {
    /* Do nothing if capture is not started */
    if (!m_isRunning)
    {
      m_logger->warn("Capture was triggered but capture is not started");
      return;
    }

    m_logger->trace("Capture triggered");
    LoggingService::Instance().flush();

    static const size_t NUM_ATTEMPTS = 5;
    size_t attempts;

    /* Grab point cloud */
    CloudPtr cloud;
    attempts = 0;
    while (!cloud && attempts < NUM_ATTEMPTS)
    {
      m_logger->trace("Grabbing point cloud, attempt {}", attempts);
      cloud = m_cloudGrabber->grabCloud();
      attempts++;
    }

    if (!cloud)
    {
      m_logger->error("Failed to grab point cloud, frame will be skipped!");
      return;
    }

    /* Grab IMU frame */
    IMUFrame_sptr imu;
    if (m_imuGrabber)
    {
      attempts = 0;
      while (!imu && attempts < NUM_ATTEMPTS)
      {
        m_logger->trace("Grabbing IMU frame, attempt {}", attempts);
        imu = m_imuGrabber->grabFrame();
        attempts++;
      }

      if (!imu)
      {
        m_logger->error("Failed to grab IMU frame, frame will be skipped!");
        return;
      }
    }
    else
    {
      m_logger->info("No IMU grabber defined");
    }

    /* Start post capture operations */
    m_logger->trace("Starting post capture operations");
    for (auto it = m_postCaptureOperations.begin(); it != m_postCaptureOperations.end(); ++it)
    {
      // TODO: run each operation is own thread
      (*it)->process(m_currentFrameCount, cloud, imu);
    }

    /* Increment frame counter */
    m_currentFrameCount++;

    LoggingService::Instance().flush();
  }

  std::ostream &operator<<(std::ostream &s, const CaptureController &o)
  {
    s << "CloudCapture["
      << "cloud grabber = " << typeid(*(o.m_cloudGrabber)).name()
      << ", IMU grabber = " << (o.m_imuGrabber ? typeid(*(o.m_imuGrabber)).name() : "none")
      << ", capture triggers = [";

    for (auto it = o.m_captureTriggers.begin(); it != o.m_captureTriggers.end();)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_captureTriggers.end())
        s << ", ";
    }

    s << "], post processing = [";

    for (auto it = o.m_postCaptureOperations.begin(); it != o.m_postCaptureOperations.end(); ++it)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_postCaptureOperations.end())
        s << ", ";
    }

    s << "]]";

    return s;
  }
}
}
