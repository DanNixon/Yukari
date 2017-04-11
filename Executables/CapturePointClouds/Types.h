/** @file */

#pragma once

#include <pcl/point_types.h>

#include <YukariCloudCapture/ICloudGrabber.h>

namespace Yukari
{
  namespace CaptureApp
  {
    typedef CloudCapture::ICloudGrabber<pcl::PointXYZRGBA> CloudGrabber;
    typedef typename CloudGrabber::Ptr CloudGrabberPtr;
  }
}