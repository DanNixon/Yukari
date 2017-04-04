/** @file */

#pragma once

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Yukari
{
namespace CloudCapture
{
  class ICloudGrabber
  {
  public:
    typedef pcl::PointXYZRGBA PointType;
    typedef pcl::PointCloud<PointType> Cloud;

  public:
    virtual void open()
    {
    }

    virtual void close()
    {
    }

    virtual bool isOpen() const
    {
      return true;
    }

    virtual Cloud::ConstPtr grabCloud() = 0;
  };

  typedef std::shared_ptr<ICloudGrabber> ICloudGrabber_sptr;
}
}
