/** @file */

#pragma once

#include "MSPGrabber.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace IMU
{
  class TeensyIMUDevice : public MSPGrabber
  {
  public:
    TeensyIMUDevice(const std::string &port, unsigned int baud = 115200);
    virtual ~TeensyIMUDevice();

    virtual IMUFrame::Ptr grabFrame() override;

  protected:
    MSP::MSPClient::Payload m_mspPayloadQuat;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
