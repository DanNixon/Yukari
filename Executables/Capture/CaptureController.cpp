/** @file */

#include "CaptureController.h"

#include <typeinfo>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <YukariIMU/IMUFrame.h>

using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Triggers;

namespace Yukari
{
namespace Capture
{
  CaptureController::CaptureController()
      : m_logger(LoggingService::Instance().getLogger("CaptureController"))
      , m_outlierRemoval(true)
      , m_outlierRemovalMeanK(50)
      , m_outlierRemovalStdDevMulThr(1.0)
  {
    m_isRunning.store(false);
  }

  int CaptureController::run()
  {
    start();

    /* Wait for termination signal */
    while (m_isRunning.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    stop();

    return 0;
  }

  void CaptureController::addCaptureTrigger(ITrigger::Ptr trigger)
  {
    trigger->setHandler([this]() { triggerCapture(); });
    m_captureTriggers.push_back(trigger);
    m_logger->debug("Added capture trigger");
  }

  void CaptureController::addExitTrigger(ITrigger::Ptr trigger)
  {
    trigger->setHandler([this]() {
      m_logger->info("Got exit trigger, exiting...");
      m_isRunning.store(false);
    });

    m_exitTriggers.push_back(trigger);
    m_logger->debug("Added exit trigger");
  }

  bool CaptureController::start()
  {
    if (m_isRunning.load())
    {
      m_logger->error("Already running");
      return false;
    }

    if (m_exitTriggers.empty())
      m_logger->warn("No exit triggers defined");

    /* Enable exit triggers */
    m_logger->info("Enabling exit triggers");
    for (auto it = m_exitTriggers.begin(); it != m_exitTriggers.end(); ++it)
      (*it)->enable();

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

    /* Check if IMU grabber provides a capture trigger */
    if (m_imuGrabber)
    {
      ITrigger::Ptr imuGrabberTrigger = m_imuGrabber->trigger();
      if (imuGrabberTrigger)
      {
        m_logger->info("Adding a capture trigger provided by IMU grabber");
        addCaptureTrigger(imuGrabberTrigger);
      }
    }

    /* Reset frame count */
    m_currentFrameCount = 0;

    /* Start operation workers */
    m_logger->trace("Starting post capture operation workers");
    for (auto it = m_postCaptureOperations.begin(); it != m_postCaptureOperations.end(); ++it)
      (*it)->start();

    /* Check if cloud grabber provides a capture trigger */
    ITrigger::Ptr cloudGrabberTrigger = m_cloudGrabber->trigger();
    if (cloudGrabberTrigger)
    {
      m_logger->info("Adding a capture trigger provided by cloud grabber");
      addCaptureTrigger(cloudGrabberTrigger);
    }

    /* Enable capture triggers */
    m_logger->info("Enabling capture triggers");
    for (auto it = m_captureTriggers.begin(); it != m_captureTriggers.end(); ++it)
      (*it)->enable();

    /* Open cloud grabber */
    m_logger->info("Opening cloud grabber");
    m_cloudGrabber->open();
    if (!m_cloudGrabber->isOpen())
    {
      m_logger->error("Failed to open cloud grabber!");
      return false;
    }
    m_logger->debug("Cloud grabber opened");

    /* Set running flags */
    m_isRunning.store(true);

    m_logger->info("Capture started");
    LoggingService::Instance().flush();

    return true;
  }

  bool CaptureController::stop()
  {
    /* Clear running flag */
    m_isRunning.store(false);

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

    /* Stop operation workers */
    m_logger->trace("Stopping post capture operation workers");
    for (auto it = m_postCaptureOperations.begin(); it != m_postCaptureOperations.end(); ++it)
      (*it)->stop(!m_forceExit);

    m_logger->info("Capture stopped");
    LoggingService::Instance().flush();

    return true;
  }

  void CaptureController::triggerCapture()
  {
    /* Do nothing if capture is not started */
    if (!m_isRunning.load())
    {
      m_logger->warn("Capture was triggered but capture is not started");
      return;
    }

    m_logger->info("Frame capture triggered for frame {}", m_currentFrameCount);
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
    IMUFrame::Ptr imu;
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

    /* Filter NaN alues from point cloud */
    m_logger->trace("Removing NaN values from captured cloud");
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }

    /* Filter outliers */
    if (m_outlierRemoval)
    {
      m_logger->trace("Performing statistical outlier point removal");
      pcl::StatisticalOutlierRemoval<PointType> sor;
      sor.setMeanK(m_outlierRemovalMeanK);
      sor.setStddevMulThresh(m_outlierRemovalStdDevMulThr);
      size_t previousPointCount = cloud->size();
      sor.filter(*cloud);
      m_logger->debug("Removed {} outliers ({} points after filtering)", previousPointCount - cloud->size(), cloud->size());
    }

    /* Queue post capture operations */
    m_logger->trace("Queueing post capture operations");
    for (auto it = m_postCaptureOperations.begin(); it != m_postCaptureOperations.end(); ++it)
      (*it)->postTask({m_currentFrameCount, cloud, imu});

    /* Increment frame counter */
    m_currentFrameCount++;

    LoggingService::Instance().flush();
    m_logger->debug("Frame capture done");
  }

  std::ostream &operator<<(std::ostream &s, const CaptureController &o)
  {
    s << "CloudCapture["
      << "cloud grabber = " << typeid(*(o.m_cloudGrabber)).name()
      << ", IMU grabber = " << (o.m_imuGrabber ? typeid(*(o.m_imuGrabber)).name() : "none")
      << ", capture triggers = [";

    for (auto it = o.m_captureTriggers.cbegin(); it != o.m_captureTriggers.cend();)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_captureTriggers.cend())
        s << ", ";
    }

    s << "], post processing = [";

    for (auto it = o.m_postCaptureOperations.cbegin(); it != o.m_postCaptureOperations.cend();)
    {
      s << typeid(*(*(it++))).name();
      if (it != o.m_postCaptureOperations.cend())
        s << ", ";
    }

    s << "]]";

    return s;
  }
}
}
