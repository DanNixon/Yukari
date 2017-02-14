#pragma once

#include <memory>

#include <pcl/visualization/pcl_visualizer.h>

#include <YukariCloudCapture/ICloudGrabber.h>

namespace Yukari
{
namespace CloudGrabberTestApp
{
  class CloudGrabberVisualisation
  {
  public:
    CloudGrabberVisualisation(Yukari::CloudCapture::ICloudGrabber_sptr grabber);

    void run();

  private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_cloudViewer;
    Yukari::CloudCapture::ICloudGrabber_sptr m_grabber;
  };
}
}
