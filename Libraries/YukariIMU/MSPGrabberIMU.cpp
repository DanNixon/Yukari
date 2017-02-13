/** @file */

#include "MSPGrabberIMU.h"

namespace Yukari
{
namespace IMU
{
  MSPGrabberIMU::MSPGrabberIMU(const std::string &port, unsigned int baud)
      : MSPGrabber(port, baud)
  {
    m_mspPayload.reserve(9);
  }

  MSPGrabberIMU::~MSPGrabberIMU()
  {
  }

  IMUFrame_sptr MSPGrabberIMU::grabFrame()
  {
    m_mspPayload.clear();

    bool ok = m_client.requestData(MSPClient::RAW_IMU, m_mspPayload) &&
              MSPClient::ParseRawIMUPayload(m_mspPayload, m_gyro, m_acc, m_mag);

    if (!ok)
      return nullptr;

    auto timeNow = std::chrono::high_resolution_clock::now();
    auto frameDuration = timeNow - m_lastFrameTime;
    m_lastFrameTime = timeNow;

    auto retVal = std::make_shared<IMUFrame>(frameDuration);

    /* TODO */

    return retVal;
  }
}
}
