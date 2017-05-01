/** @file */

#pragma once

#include "ISerialGrabber.h"

#include <chrono>
#include <memory>

#include <YukariMSP/MSPClient.h>

namespace Yukari
{
namespace IMU
{
  class IMSPGrabber : public ISerialGrabber
  {
  public:
    typedef std::shared_ptr<IMSPGrabber> Ptr;

  public:
    IMSPGrabber(const std::string &port, unsigned int baud = 115200);
    virtual ~IMSPGrabber();

    virtual void open() override;

    virtual IMUFrame::Ptr grabFrame() override = 0;

    bool calibrateAccelerometer();
    bool calibrateMagnetometer();

  protected:
    serial::Timeout m_calibrationTimeout;
    MSP::MSPClient m_client;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;

    MSP::MSPClient::Payload m_mspPayload;
  };
}
}
