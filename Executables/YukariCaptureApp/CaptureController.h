/** @file */

#pragma once

#include <memory>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariIMU/IIMUGrabber.h>

namespace Yukari
{
namespace CaptureApp
{
  class CaptureController
  {
  public:
    CaptureController();

    int run();

  private:
    Yukari::CloudCapture::ICloudGrabber_sptr m_cloudGrabber;
    Yukari::IMU::IIMUGrabber_sptr m_imuGrabber;
  };

  typedef std::shared_ptr<CaptureController> CaptureController_sptr;
}
}
