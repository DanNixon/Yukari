/** @file */

#pragma once

#include <pcl/point_types.h>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariProcessing/IFrameProcessingTask.h>

namespace Yukari
{
namespace Capture
{
  typedef pcl::PointXYZRGBA PointType;

  typedef CloudCapture::ICloudGrabber<PointType> CloudGrabber;
  typedef typename CloudGrabber::Ptr CloudGrabberPtr;

  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  typedef Processing::IFrameProcessingTask::Ptr ProcessingTaskPtr;
}
}
