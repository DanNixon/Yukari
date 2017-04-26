/** @file */

#pragma once

#include <memory>

#include <pcl/point_cloud.h>

#include <YukariTriggers/ITrigger.h>

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE> class ICloudGrabber
  {
  public:
    typedef std::shared_ptr<ICloudGrabber<POINT_TYPE>> Ptr;
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

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

    virtual Triggers::ITrigger::Ptr trigger()
    {
      return nullptr;
    }

    virtual CloudPtr grabCloud() = 0;
  };
}
}
