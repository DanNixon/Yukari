/** @file */

#pragma once

#include <memory>

#include <YukariIMU/IMUFrame.h>

#include "Types.h"

namespace Yukari
{
namespace CaptureApp
{
  class IPostCaptureTask
  {
  public:
    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) = 0;
  };

  typedef std::shared_ptr<IPostCaptureTask> IPostCaptureTask_sptr;
}
}
