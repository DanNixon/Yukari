/** @file */

#include "IMSPGrabber.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::MSP;

namespace Yukari
{
namespace IMU
{
  IMSPGrabber::IMSPGrabber(const std::string &port, unsigned int baud)
      : ISerialGrabber(port, baud)
      , m_calibrationTimeout(serial::Timeout::simpleTimeout(30000))
      , m_client(m_port)
  {
  }

  IMSPGrabber::~IMSPGrabber()
  {
    close();
  }

  void IMSPGrabber::open()
  {
    ISerialGrabber::open();
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  bool IMSPGrabber::calibrateAccelerometer()
  {
    auto logger = LoggingService::Instance().getLogger("runFrameGrabber");

    if (!m_port.isOpen())
    {
      logger->error("MSP port not open, cannot run accelerometer calibration.");
      return false;
    }

    m_port.setTimeout(m_calibrationTimeout);
    bool ok = m_client.requestData(MSPClient::ACC_CALIBRATION, m_mspPayload);
    logger->info("Accelerometer calibration done, result: {}", ok);
    m_port.setTimeout(m_defaultTimeout);

    return ok;
  }

  bool IMSPGrabber::calibrateMagnetometer()
  {
    auto logger = LoggingService::Instance().getLogger("runFrameGrabber");

    if (!m_port.isOpen())
    {
      logger->error("MSP port not open, cannot run magnetometer calibration.");
      return false;
    }

    m_port.setTimeout(m_calibrationTimeout);
    bool ok = m_client.requestData(MSPClient::MAG_CALIBRATION, m_mspPayload);
    logger->info("Magnetometer calibration done, result: {}", ok);
    m_port.setTimeout(m_defaultTimeout);

    return ok;
  }
}
}
