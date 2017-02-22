/** @file */

#pragma once

#include "MSPGrabber.h"

#include <YukariCommon/LoggingService.h>

#include "IMUConstants.h"

namespace Yukari
{
namespace IMU
{
  class MSPGrabberIMU : public MSPGrabber
  {
  public:
    MSPGrabberIMU(const std::string &port, unsigned int baud = 115200,
                  float accScale = MPU6050_1GACC_SCALE, float gyroScale = MPU6050_GYRO_SCALE,
                  float magScale = 1.0f);
    virtual ~MSPGrabberIMU();

    virtual void open() override;
    virtual IMUFrame_sptr grabFrame() override;

  protected:
    const float m_accScale;
    const float m_gyroScale;
    const float m_magScale;

    MSPClient::Payload m_mspPayloadIMU;

    float m_attitude[3];
    int16_t m_gyro[3];
    int16_t m_acc[3];
    int16_t m_mag[3];

    Yukari::Maths::Vector3 m_positionAccum;

  private:
    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
