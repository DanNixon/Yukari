/** @file */

#pragma once

#include <memory>

#include <pcl/point_cloud.h>

namespace Yukari
{
namespace CloudCapture
{
  class ICloudGrabber
  {
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

    template <typename POINT_TYPE>
    virtual pcl::PointCloud<POINT_TYPE>::ConstPtr grabCloud() = 0;
  };

  typedef std::shared_ptr<ICloudGrabber> ICloudGrabber_sptr;
}
}
