/** @file */

#pragma once

#include "PCLCloudGrabberWrapper.h"

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE>
  class PCDFileCloudGrabber : public PCLCloudGrabberWrapper<POINT_TYPE>
  {
  public:
    PCDFileCloudGrabber()
        : PCLCloudGrabberWrapper(nullptr)
    {
      /* TODO */
    }

    virtual ~PCDFileCloudGrabber()
    {
    }
  };
}
}
