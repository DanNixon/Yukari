#include "CloudGrabberVisualisation.h"

namespace Yukari
{
namespace CloudGrabberTestApp
{
  CloudGrabberVisualisation::CloudGrabberVisualisation(
      Yukari::CloudCapture::ICloudGrabber_sptr grabber)
      : m_cloudViewer(new pcl::visualization::PCLVisualizer("Cloud grabber visualisation"))
      , m_grabber(grabber)
  {
  }

  void CloudGrabberVisualisation::run()
  {
    m_cloudViewer->setCameraFieldOfView(1.02259994f);

    m_grabber->open();

    bool cloudInit = false;
    while (!m_cloudViewer->wasStopped())
    {
      m_cloudViewer->spinOnce();

      Yukari::CloudCapture::ICloudGrabber::Cloud::ConstPtr cloud = m_grabber->grabCloud();
      if (cloud)
      {
        if (!cloudInit)
        {
          m_cloudViewer->setPosition(0, 0);
          m_cloudViewer->setSize(cloud->width, cloud->height);
          cloudInit = !cloudInit;
        }

        if (!m_cloudViewer->updatePointCloud(cloud, "OpenNICloud"))
        {
          m_cloudViewer->addPointCloud(cloud, "OpenNICloud");
          m_cloudViewer->resetCameraViewpoint("OpenNICloud");
          m_cloudViewer->setCameraPosition(0, 0, 0,   // Position
                                           0, 0, 1,   // Viewpoint
                                           0, -1, 0); // Up
        }
      }
    }

    m_grabber->close();
  }
}
}