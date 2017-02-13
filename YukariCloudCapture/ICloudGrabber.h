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
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

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

    virtual Cloud::ConstPtr getCloud() = 0;
  };

  typedef std::shared_ptr<ICloudGrabber> ICloudGrabber_sptr;
}
}
