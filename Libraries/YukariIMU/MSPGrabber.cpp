/** @file */

#include "MSPGrabber.h"

#include <YukariCommon/LoggingService.h>
#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::MSP;

namespace Yukari
{
namespace IMU
{
  MSPGrabber::MSPGrabber(const std::string &port, unsigned int baud)
      : m_defaultTimeout(serial::Timeout::simpleTimeout(1000))
      , m_calibrationTimeout(serial::Timeout::simpleTimeout(30000))
      , m_port(port, baud, m_defaultTimeout)
      , m_client(m_port)
  {
  }

  MSPGrabber::~MSPGrabber()
  {
    close();
  }

  void MSPGrabber::open()
  {
    if (!m_port.isOpen())
      m_port.open();

    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  void MSPGrabber::close()
  {
    if (m_port.isOpen())
      m_port.close();
  }

  bool MSPGrabber::isOpen() const
  {
    return m_port.isOpen();
  }

  bool MSPGrabber::calibrateAccelerometer()
  {
    auto logger = LoggingService::GetLogger("runFrameGrabber");

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

  bool MSPGrabber::calibrateMagnetometer()
  {
    auto logger = LoggingService::GetLogger("runFrameGrabber");

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
