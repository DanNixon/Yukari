/** @file */

#include "CaptureController.h"

#include <typeinfo>

#include <pcl/io/pcd_io.h>

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

    /* Ensure capture directory exists */
    m_logger->info("Capture path: {}", m_outputDirectory);
    boost::filesystem::create_directories(m_outputDirectory);
    m_logger->debug("Capture root directory created");

    /* Reset frrame count */
    m_currentFrameCount = 0;

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

    /* Grab point cloud */
    auto cloud = m_cloudGrabber->grabCloud();
    if (!cloud)
    {
      m_logger->error("Failed to grab point cloud, frame will be skipped!");
      return;
    }

    /* Grab IMU frame */
    IMUFrame_sptr imu;
    if (m_imuGrabber)
    {
      imu = m_imuGrabber->grabFrame();
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

    /* Save cloud */
    boost::filesystem::path cloudFilename =
        m_outputDirectory / (std::to_string(m_currentFrameCount) + "_cloud.pcd");

    m_logger->trace("Saving point cloud for frame {}: {}", m_currentFrameCount, cloudFilename);
    pcl::io::savePCDFileASCII(cloudFilename.string(), *cloud);

    /* Save IMU frame */
    if (m_imuGrabber)
    {
      boost::filesystem::path imuFilename =
          m_outputDirectory / (std::to_string(m_currentFrameCount) + "_imu.txt");

      m_logger->trace("Saving IMU frame for frame {}: {}", m_currentFrameCount, imuFilename);

      std::ofstream imuFile;
      imuFile.open(imuFilename.string());
      imuFile << *imu << '\n';
      imuFile.close();
    }

    /* Increment frame counter */
    m_currentFrameCount++;

    LoggingService::Instance().flush();
  }

  std::ostream &operator<<(std::ostream &s, const CaptureController &o)
  {
    s << "CloudCapture[out dir = " << o.m_outputDirectory
      << ", cloud grabber = " << typeid(*(o.m_cloudGrabber)).name()
      << ", IMU grabber = " << (o.m_imuGrabber ? typeid(*(o.m_imuGrabber)).name() : "none")
      << ", capture triggers = [";

    for (auto it = o.m_captureTriggers.begin(); it != o.m_captureTriggers.end();)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_captureTriggers.end())
        s << ", ";
    }

    s << "]]";

    return s;
  }
}
}
