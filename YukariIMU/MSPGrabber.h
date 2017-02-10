/** @file */

#pragma once

#include "IGrabber.h"

#include <chrono>
#include <serial/serial.h>

#include "MSPClient.h"

namespace Yukari
{
namespace IMU
{
  class MSPGrabber : public IGrabber
  {
  public:
    MSPGrabber(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabber();

    virtual void open();
    virtual void close();
    virtual bool isOpen() const;

    virtual IMUFrame_sptr grabFrame() = 0;

    bool calibrateAccelerometer();
    bool calibrateMagnetometer();

  protected:
    serial::Timeout m_defaultTimeout;
    serial::Timeout m_calibrationTimeout;
    serial::Serial m_port;
    MSPClient m_client;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;

    MSPClient::Payload m_mspPayload;
  };
}
}
