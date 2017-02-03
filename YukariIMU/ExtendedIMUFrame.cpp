/** @file */

#include "ExtendedIMUFrame.h"

namespace Yukari
{
namespace IMU
{
  ExtendedIMUFrame::ExtendedIMUFrame()
  {
  }

  IMUFrame_sptr ExtendedIMUFrame::predictFrame(float secondsFromNow) const
  {
    /* TODO */
    return std::make_shared<IMUFrame>();
  }
}
}
