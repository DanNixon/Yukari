/** @file */

#pragma once

#include "IIMUGrabber.h"

#include <chrono>
#include <memory>
#include <serial/serial.h>

#include "MSPClient.h"

namespace Yukari
{
namespace IMU
{
  class MSPGrabber : public IIMUGrabber
  {
  public:
    MSPGrabber(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabber();

    virtual void open() override;
    virtual void close() override;
    virtual bool isOpen() const override;

    virtual IMUFrame_sptr grabFrame() override = 0;

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

  typedef std::shared_ptr<MSPGrabber> MSPGrabber_sptr;
}
}
