/** @file */

#include "CaptureController.h"

#include <typeinfo>

#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController::CaptureController()
      : m_logger(LoggingService::GetLogger("CaptureController"))
      , m_isRunning(false)
      , m_shouldStop(false)
  {
  }

  int CaptureController::run()
  {
    /* Start capture */
    if (!start())
    {
      m_logger->error("Failed to start capture");
      LoggingService::Flush();
      return 1;
    }

    /* Wait for termination signal */
    while (m_isRunning)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      if (m_shouldStop)
      {
        stop();
        LoggingService::Flush();
      }
    }

    return 0;
  }

  bool CaptureController::start()
  {
    /* Open IMU grabber */
    m_logger->info("Opening IMU grabber");
    m_imuGrabber->open();
    if (!m_imuGrabber->isOpen())
    {
      m_logger->error("Failed to open IMU grabber!");
      return false;
    }
    m_logger->debug("IMU grabber opened");

    /* Open cloud grabber */
    m_logger->info("Opening cloud grabber");
    m_cloudGrabber->open();
    if (!m_cloudGrabber->isOpen())
    {
      m_logger->error("Failed to open cloud grabber!");
      return false;
    }
    m_logger->debug("Cloud grabber opened");

    /* Enable exit triggers */
    m_logger->info("Enabling exit triggers");
    for (auto it = m_exitTriggers.begin(); it != m_exitTriggers.end(); ++it)
      (*it)->enable();

    /* Enable capture triggers */
    m_logger->info("Enabling capture triggers");
    for (auto it = m_captureTriggers.begin(); it != m_captureTriggers.end(); ++it)
      (*it)->enable();

    /* Generate capture root path name */
    auto time = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *facet = new boost::posix_time::time_facet();
    facet->format("%Y-%m-%dT%H_%M_%S");

    std::stringstream stream;
    stream.imbue(std::locale(std::locale::classic(), facet));
    stream << time;

    m_currentCaptureRootPath = m_outputRootPath / stream.str();
    m_logger->info("Capture root path: {}", m_currentCaptureRootPath);

    /* Ensure capture directory exists */
    boost::filesystem::create_directories(m_currentCaptureRootPath);
    m_logger->debug("Capture root directory created");

    /* Reset frrame count */
    m_currentFrameCount = 0;

    /* Set running flags */
    m_isRunning = true;
    m_shouldStop = false;

    return true;
  }

  bool CaptureController::stop()
  {
    /* Clear running flag */
    m_isRunning = false;

    /* Disable exit triggers */
    m_logger->info("Disabling exit triggers");
    for (auto it = m_exitTriggers.begin(); it != m_exitTriggers.end(); ++it)
      (*it)->disable();

    /* Disable capture triggers */
    m_logger->info("Disabling capture triggers");
    for (auto it = m_captureTriggers.begin(); it != m_captureTriggers.end(); ++it)
      (*it)->disable();

    /* Close IMU grabber */
    m_logger->info("Closing IMU grabber");
    m_imuGrabber->close();
    if (m_imuGrabber->isOpen())
    {
      m_logger->error("Failed to close IMU grabber!");
      return false;
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

  void CaptureController::addExitTrigger(ITrigger_sptr trigger)
  {
    trigger->setHandler([this]() { markShouldStop(); });
    m_exitTriggers.push_back(trigger);
    m_logger->debug("Added exit trigger");
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
    LoggingService::Flush();

    /* Generate output filenames */
    boost::filesystem::path cloudFilename =
        m_currentCaptureRootPath / (std::to_string(m_currentFrameCount) + "_cloud.pcd");
    boost::filesystem::path imuFilename =
        m_currentCaptureRootPath / (std::to_string(m_currentFrameCount) + "_imu.txt");

    /* Grab point cloud */
    auto cloud = m_cloudGrabber->grabCloud();
    if (!cloud)
    {
      m_logger->error("Failed to grab point cloud, frame will be skipped!");
      return;
    }

    /* Grab IMU frame */
    auto imu = m_imuGrabber->grabFrame();
    if (!imu)
    {
      m_logger->error("Failed to grab IMU frame, frame will be skipped!");
      return;
    }

    /* Save cloud */
    m_logger->trace("Saving point cloud for frame {}: {}", m_currentFrameCount, cloudFilename);
    pcl::io::savePCDFileASCII(cloudFilename.string(), *cloud);

    /* Save IMU frame */
    m_logger->trace("Saving IMU frame for frame {}: {}", m_currentFrameCount, imuFilename);
    std::ofstream imuFile;
    imuFile.open(imuFilename.string());
    imuFile << *imu << '\n';
    imuFile.close();

    /* Increment frame counter */
    m_currentFrameCount++;

    LoggingService::Flush();
  }

  void CaptureController::markShouldStop()
  {
    m_shouldStop = true;
  }

  std::ostream &operator<<(std::ostream &s, const CaptureController &o)
  {
    s << "CloudCapture[out dir = " << o.m_outputRootPath
      << ", cloud grabber = " << typeid(*(o.m_cloudGrabber)).name()
      << ", IMU grabber = " << typeid(*(o.m_imuGrabber)).name() << ", capture triggers = [";

    for (auto it = o.m_captureTriggers.begin(); it != o.m_captureTriggers.end();)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_captureTriggers.end())
        s << ", ";
    }

    s << "], exit triggers = [";

    for (auto it = o.m_exitTriggers.begin(); it != o.m_exitTriggers.end();)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_exitTriggers.end())
        s << ", ";
    }

    s << "]]";

    return s;
  }
}
}
