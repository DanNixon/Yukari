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
  class MSPGrabberAttitude : public IGrabber
  {
  public:
    MSPGrabberAttitude(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabberAttitude();

    virtual void open();
    virtual void close();
    virtual bool isOpen() const;

    virtual IMUFrame_sptr grabFrame();

  protected:
    serial::Serial m_port;
    MSPClient m_client;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;

    float m_attitude[3];
    MSPClient::Payload m_mspPayload;
  };
}
}
