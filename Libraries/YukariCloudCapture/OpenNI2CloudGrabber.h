/** @file */

#pragma once

#include "PCLCloudGrabberWrapper.h"

#include <pcl/io/openni2/openni.h>
#include <pcl/io/openni2_grabber.h>

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE>
  class OpenNI2CloudGrabber : public PCLCloudGrabberWrapper<POINT_TYPE>
  {
  public:
    OpenNI2CloudGrabber(const std::string &device, pcl::io::OpenNI2Grabber::Mode depthMode, pcl::io::OpenNI2Grabber::Mode imageMode)
      : PCLCloudGrabberWrapper(std::make_shared<pcl::io::OpenNI2Grabber>(device, depthMode, imageMode))
    {
      m_cloudTransform(0, 0) = -1.0f;
      m_cloudTransform(1, 1) = -1.0f;
      m_cloudTransform(2, 2) = -1.0f;
    }

    virtual ~OpenNI2CloudGrabber()
    {
    }
  };
}
}
