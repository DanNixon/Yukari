/** @file */

#pragma once

#include "MSPGrabber.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace IMU
{
  class MSPGrabberAttitude : public MSPGrabber
  {
  public:
    MSPGrabberAttitude(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabberAttitude();

    virtual IMUFrame_sptr grabFrame() override;

  protected:
    float m_attitude[3];

  private:
    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
