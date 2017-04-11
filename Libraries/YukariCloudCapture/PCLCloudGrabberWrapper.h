/** @file */

#pragma once

#include "ICloudGrabber.h"

#include <mutex>

#include <pcl/io/openni2/openni.h>
#include <pcl/io/openni2_grabber.h>

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE>
  class PCLCloudGrabberWrapper : public ICloudGrabber
  {
  public:
    PCLCloudGrabberWrapper(std::shared_ptr<pcl::Grabber> grabber, Eigen::Matrix4f transform = Eigen::Matrix4f::Identity())
      : m_grabber(grabber)
      , m_cloudTransform(transform)
    {
      boost::function<void(const ICloudGrabber::Cloud::ConstPtr &)> cloudCB =
        boost::bind(&PCLCloudGrabberWrapper::cloudCallback, this, _1);
      m_cloudCBConnection = m_grabber.registerCallback(cloudCB);
    }

    virtual ~PCLCloudGrabberWrapper()
    {
    }

    virtual void open() override
    {
      m_grabber->start();
    }

    virtual void close() override
    {
      m_grabber->stop();
    }

    virtual bool isOpen() const override
    {
      return m_grabber.isRunning();
    }

    virtual pcl::PointCloud<POINT_TYPE>::ConstPtr grabCloud() override
    {
      pcl::PointCloud<POINT_TYPE>::ConstPtr rawCloud;

      /* Get captured cloud */
      if (m_rawCloudMutex.try_lock())
      {
        m_rawCloud.swap(rawCloud);
        m_rawCloudMutex.unlock();
      }

      if (!rawCloud)
        return nullptr;

      /* Transform captured cloud */
      pcl::PointCloud<POINT_TYPE>::Ptr transformedCloud(new pcl::PointCloud<POINT_TYPE>());
      pcl::transformPointCloud(*rawCloud, *transformedCloud, m_cloudTransform);

      return transformedCloud;
    }

  private:
    void cloudCallback(pcl::PointCloud<POINT_TYPE>::ConstPtr cloud)
    {
      std::lock_guard<std::mutex> lock(m_rawCloudMutex);
      m_rawCloud = cloud;
    }

  protected:
    Eigen::Matrix4f m_cloudTransform;

    std::shared_ptr<pcl::Grabber> m_grabber;
    boost::signals2::connection m_cloudCBConnection;

    std::mutex m_rawCloudMutex;
    pcl::PointCloud<POINT_TYPE>::ConstPtr m_rawCloud;
  };
}
}
