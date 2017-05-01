/** @file */

#include "STM32IMUDevice.h"

using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  STM32IMUDevice::STM32IMUDevice(const std::string &port, unsigned int baud)
      : ISerialGrabber(port, baud)
  {
  }

  STM32IMUDevice::~STM32IMUDevice()
  {
  }

  IMUFrame::Ptr STM32IMUDevice::grabFrame()
  {
    auto retVal = std::make_shared<IMUFrame>();

    // TODO

    return retVal;
  }
}
}
