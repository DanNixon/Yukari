/** @file */

#pragma once

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariIMU/IIMUGrabber.h>

namespace Yukari
{
namespace CaptureApp
{
  class CaptureController
  {

  private:
    Yukari::CloudCapture::ICloudGrabber_sptr m_cloudGrabber;
    Yukari::IMU::IIMUGrabber_sptr m_imuGrabber;
  };
}
}
