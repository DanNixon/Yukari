/** @file */

#include "CaptureController.h"

#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController::CaptureController()
      : m_logger(LoggingService::GetLogger("CaptureController"))
  {
  }

  int CaptureController::run()
  {
    if (!start())
      return 1;

    // TODO

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

    /* Open cloud grabber */
    m_logger->info("Opening cloud grabber");
    m_cloudGrabber->open();
    if (!m_cloudGrabber->isOpen())
    {
      m_logger->error("Failed to open cloud grabber!");
      return false;
    }

    /* TODO */

    m_isRunning = true;

    return true;
  }

  bool CaptureController::stop()
  {
    m_isRunning = false;

    /* TODO */

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

  void CaptureController::triggerCapture()
  {
    /* TODO */
	boost::filesystem::path cloudFile;
	boost::filesystem::path imuFile;

    /* Grab data */
    auto cloud = m_cloudGrabber->grabCloud();
    auto imu = m_imuGrabber->grabFrame();

    /* Save cloud */
    /* TODO */
	//pcl::io::savePCDFileASCII(cloudFilename, cloud);

    /* Save IMU frame */
    /* TODO */
  }
}
}
