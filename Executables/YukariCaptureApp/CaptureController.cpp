/** @file */

#include "CaptureController.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController::CaptureController()
    : m_logger(LoggingService::getLogger("CaptureController"))
  {
    // TODO
  }

  int CaptureController::run()
  {
    /* Open IMU grabber */
    m_logger->info("Opening IMU grabber");
    m_imuGrabber->open();
    if (!m_imuGrabber->isOpen())
    {
      m_logger->error("Failed to open IMU grabber!");
      return 2;
    }

    /* Open cloud grabber */
    m_logger->info("Opening cloud grabber");
    m_cloudGrabber->open();
    if (!m_cloudGrabber->isOpen())
    {
      m_logger->error("Failed to open cloud grabber!");
      return 2;
    }

    // TODO
    return 0;
  }
}
}
