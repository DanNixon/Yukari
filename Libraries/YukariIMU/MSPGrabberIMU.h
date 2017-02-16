/** @file */

#pragma once

#include "MSPGrabber.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace IMU
{
  class MSPGrabberIMU : public MSPGrabber
  {
  public:
    MSPGrabberIMU(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabberIMU();

    virtual IMUFrame_sptr grabFrame() override;

  protected:
    MSPClient::Payload m_mspPayloadIMU;

    float m_attitude[3];
    int16_t m_gyro[3];
    int16_t m_acc[3];
    int16_t m_mag[3];

    /* TODO */
    float m_y;

  private:
    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
