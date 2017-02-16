/** @file */

#include "MSPGrabberIMU.h"

#include <boost/qvm/all.hpp>

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

using namespace boost::qvm;
using namespace Yukari::Common;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  MSPGrabberIMU::MSPGrabberIMU(const std::string &port, unsigned int baud)
      : MSPGrabber(port, baud)
      , m_logger(LoggingService::GetLogger("MSPGrabberIMU"))
  {
    m_mspPayload.reserve(6);
    m_mspPayloadIMU.reserve(9);

    /* TODO */
    m_y = 0.0f;
  }

  MSPGrabberIMU::~MSPGrabberIMU()
  {
  }

  IMUFrame_sptr MSPGrabberIMU::grabFrame()
  {
    m_mspPayload.clear();
    m_mspPayloadIMU.clear();

    /* MSP data */
    bool ok = m_client.requestData(MSPClient::RAW_IMU, m_mspPayloadIMU) &&
              MSPClient::ParseRawIMUPayload(m_mspPayloadIMU, m_gyro, m_acc, m_mag) &&
              m_client.requestData(MSPClient::ATTITUDE, m_mspPayload) &&
              MSPClient::ParseAttitudePayload(m_mspPayload, m_attitude);

    if (!ok)
    {
      m_logger->error("Failed to get all MSP data");
      return nullptr;
    }

    /* Calculate timestep */
    auto timeNow = std::chrono::high_resolution_clock::now();
    auto frameDuration = timeNow - m_lastFrameTime;
    m_lastFrameTime = timeNow;

    auto retVal = std::make_shared<IMUFrame>(frameDuration);

    /* Set orientation from attitude data */
    Quaternion x, y, z;
    rotate_x(x, -m_attitude[2] * DEG_TO_RAD);
    rotate_y(y, -m_attitude[0] * DEG_TO_RAD);
    rotate_z(z, -m_attitude[1] * DEG_TO_RAD);
    retVal->orientation() = x * y * z;

    /* Position */
    /* TODO */
    retVal->position() = Vector3(0, m_y, 0);
    m_y += 0.001f;

    return retVal;
  }
}
}
