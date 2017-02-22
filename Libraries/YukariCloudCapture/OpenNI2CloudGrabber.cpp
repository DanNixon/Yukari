/** @file */

#include "OpenNI2CloudGrabber.h"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <thread>

namespace Yukari
{
namespace CloudCapture
{
  OpenNI2CloudGrabber::OpenNI2CloudGrabber(const std::string deviceID,
                                           const pcl::io::OpenNI2Grabber::Mode depthMode,
                                           const pcl::io::OpenNI2Grabber::Mode imageMode)
      : m_grabber(deviceID, depthMode, imageMode)
  {
    /* Init callback */
    boost::function<void(const ICloudGrabber::Cloud::ConstPtr &)> cloudCB =
        boost::bind(&OpenNI2CloudGrabber::cloudCallback, this, _1);
    m_cloudCBConnection = m_grabber.registerCallback(cloudCB);

    /* Set transformation */
    m_cloudTransform = Eigen::Matrix4f::Identity();
    m_cloudTransform(0, 0) = -1.0f;
    m_cloudTransform(1, 1) = -1.0f;
    m_cloudTransform(2, 2) = -1.0f;
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
    std::lock_guard<std::mutex> lock(m_rawCloudMutex);
    m_grabber.stop();
  }

  bool OpenNI2CloudGrabber::isOpen() const
  {
    return m_grabber.isRunning();
  }

  ICloudGrabber::Cloud::ConstPtr OpenNI2CloudGrabber::grabCloud()
  {
    ICloudGrabber::Cloud::ConstPtr rawCloud;

    /* Get captured cloud */
    if (m_rawCloudMutex.try_lock())
    {
      m_rawCloud.swap(rawCloud);
      m_rawCloudMutex.unlock();
    }

    if (!rawCloud)
      return nullptr;

    /* Transform captured cloud */
    ICloudGrabber::Cloud::Ptr transformedCloud(new ICloudGrabber::Cloud());
    pcl::transformPointCloud(*rawCloud, *transformedCloud, m_cloudTransform);

    return transformedCloud;
  }

  void OpenNI2CloudGrabber::cloudCallback(ICloudGrabber::Cloud::ConstPtr cloud)
  {
    std::lock_guard<std::mutex> lock(m_rawCloudMutex);
    m_rawCloud = cloud;
  }
}
}
