/** @file */

#pragma once

#include "ICloudGrabber.h"

#include <mutex>

#include <pcl/common/transforms.h>
#include <pcl/io/grabber.h>

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE> class PCLCloudGrabberWrapper : public ICloudGrabber<POINT_TYPE>
  {
  public:
    PCLCloudGrabberWrapper(std::shared_ptr<pcl::Grabber> grabber,
                           Eigen::Matrix4f transform = Eigen::Matrix4f::Identity())
        :m_cloudTransform(transform)
    {
      if (grabber)
        setGrabber(grabber);
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
      return m_grabber->isRunning();
    }

    virtual CloudPtr grabCloud() override
    {
      CloudConstPtr rawCloud;

      /* Get captured cloud */
      if (m_rawCloudMutex.try_lock())
      {
        m_rawCloud.swap(rawCloud);
        m_rawCloudMutex.unlock();
      }

      if (!rawCloud)
        return nullptr;

      /* Transform captured cloud */
      CloudPtr transformedCloud(new pcl::PointCloud<POINT_TYPE>());
      pcl::transformPointCloud(*rawCloud, *transformedCloud, m_cloudTransform);

      return transformedCloud;
    }

  protected:
    void setGrabber(std::shared_ptr<pcl::Grabber> grabber)
    {
      m_grabber = grabber;

      boost::function<void(const ICloudGrabber::Cloud::ConstPtr &)> cloudCB =
        boost::bind(&PCLCloudGrabberWrapper::cloudCallback, this, _1);
      m_cloudCBConnection = m_grabber->registerCallback(cloudCB);
    }

  private:
    void cloudCallback(CloudConstPtr cloud)
    {
      std::lock_guard<std::mutex> lock(m_rawCloudMutex);
      m_rawCloud = cloud;
    }

  protected:
    Eigen::Matrix4f m_cloudTransform;

    std::shared_ptr<pcl::Grabber> m_grabber;
    boost::signals2::connection m_cloudCBConnection;

    std::mutex m_rawCloudMutex;
    CloudConstPtr m_rawCloud;
  };
}
}
