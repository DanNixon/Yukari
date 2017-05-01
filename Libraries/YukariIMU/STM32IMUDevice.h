/** @file */

#pragma once

#include "ISerialGrabber.h"

#include <memory>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace IMU
{
  class STM32IMUDevice : public ISerialGrabber
  {
  public:
    typedef std::shared_ptr<STM32IMUDevice> Ptr;

    struct ValuesPacket
    {
      uint8_t header;
      uint8_t length;
      int16_t q_w;
      int16_t q_x;
      int16_t q_y;
      int16_t q_z;
      int16_t d_x;
      int16_t d_y;
      int16_t d_z;
      uint8_t checksum;
      uint8_t padding;
    };

  public:
    STM32IMUDevice(const std::string &port, unsigned int baud = 115200);
    virtual ~STM32IMUDevice();

    virtual IMUFrame::Ptr grabFrame() override;

    void calibrateAccelerometer();
    void resetDisplacement();

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
