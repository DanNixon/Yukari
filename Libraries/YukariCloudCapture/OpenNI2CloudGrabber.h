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
  class OpenNI2CloudGrabber : public ICloudGrabber
  {
  public:
    OpenNI2CloudGrabber(const std::string deviceID = "",
                        const pcl::io::OpenNI2Grabber::Mode depthMode =
                            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
                        const pcl::io::OpenNI2Grabber::Mode imageMode =
                            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode);
    virtual ~OpenNI2CloudGrabber();

    virtual void open() override;
    virtual void close() override;
    virtual bool isOpen() const override;

    virtual ICloudGrabber::Cloud::ConstPtr grabCloud() override;

  private:
    void cloudCallback(ICloudGrabber::Cloud::ConstPtr cloud);

  private:
    Eigen::Matrix4f m_cloudTransform;

    pcl::io::OpenNI2Grabber m_grabber;
    boost::signals2::connection m_cloudCBConnection;

    std::mutex m_rawCloudMutex;
    ICloudGrabber::Cloud::ConstPtr m_rawCloud;
  };
}
}
