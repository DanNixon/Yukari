/** @file */

#include "OpenNI2CloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  OpenNI2CloudGrabber::OpenNI2CloudGrabber(const std::string deviceID,
                                           const pcl::io::OpenNI2Grabber::Mode depthMode,
                                           const pcl::io::OpenNI2Grabber::Mode imageMode)
      : m_grabber(deviceID, depthMode, imageMode)
  {
    boost::function<void(const ICloudGrabber::Cloud::ConstPtr &)> cloudCB =
        boost::bind(&OpenNI2CloudGrabber::cloudCallback, this, _1);
    m_cloudCBConnection = m_grabber.registerCallback(cloudCB);
  }

  OpenNI2CloudGrabber::~OpenNI2CloudGrabber()
  {
  }

  void OpenNI2CloudGrabber::open()
  {
    m_grabber.start();
  }

  void OpenNI2CloudGrabber::close()
  {
    m_cloudMutex.try_lock();
    m_grabber.stop();
    m_cloudMutex.unlock();
  }

  bool OpenNI2CloudGrabber::isOpen() const
  {
    return m_grabber.isRunning();
  }

  ICloudGrabber::Cloud::ConstPtr OpenNI2CloudGrabber::grabCloud()
  {
    if (m_cloudMutex.try_lock())
    {
      ICloudGrabber::Cloud::ConstPtr cloud;
      m_cloud.swap(cloud);
      m_cloudMutex.unlock();

      return cloud;
    }

    return nullptr;
  }

  void OpenNI2CloudGrabber::cloudCallback(ICloudGrabber::Cloud::ConstPtr cloud)
  {
    std::lock_guard<std::mutex> lock(m_cloudMutex);
    m_cloud = cloud;
  }
}
}
