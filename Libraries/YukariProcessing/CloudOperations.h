/** @file */

#pragma once

#include <YukariCloudCapture/ICloudGrabber.h>

namespace Yukari
{
namespace Processing
{
  class CloudOperations
  {
  public:
	  CloudCapture::ICloudGrabber::Cloud::Ptr ApplyTransformationToCloud(CloudCapture::ICloudGrabber::Cloud::Ptr cloud, Eigen::Matrix4f transform);
	  CloudCapture::ICloudGrabber::Cloud::Ptr ConcatenateClouds(std::vector<CloudCapture::ICloudGrabber::Cloud::Ptr> clouds);
	  CloudCapture::ICloudGrabber::Cloud::Ptr RemoveNaNFromCloud(CloudCapture::ICloudGrabber::Cloud::Ptr cloud);
  };
}
}
