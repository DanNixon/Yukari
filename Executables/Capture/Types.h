/** @file */

#pragma once

#include <functional>

#include <pcl/point_types.h>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariIMU/IMUFrame.h>

namespace Yukari
{
namespace CaptureApp
{
  typedef pcl::PointXYZRGBA PointType;

  typedef CloudCapture::ICloudGrabber<PointType> CloudGrabber;
  typedef typename CloudGrabber::Ptr CloudGrabberPtr;

  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  typedef std::function<int(CloudConstPtr, IMU::IMUFrame_const_sptr)> PostCaptureTaskFunc;
}
}