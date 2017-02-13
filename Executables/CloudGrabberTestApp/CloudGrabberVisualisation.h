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
    CloudGrabberVisualisation(Yukari::CloudCapture::ICloudGrabber_sptr grabber)
        : m_cloudViewer(new pcl::visualization::PCLVisualizer("Cloud grabber visualisation"))
        , m_grabber(grabber)
    {
    }

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *)
    {
      if (event.getKeyCode())
        std::cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      else
        std::cout << "the special key \'" << event.getKeySym() << "\' was";

      if (event.keyDown())
        std::cout << " pressed" << std::endl;
      else
        std::cout << " released" << std::endl;
    }

    void mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *)
    {
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
          mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        std::cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY()
                  << std::endl;
      }
    }

    void run()
    {
      m_cloudViewer->registerMouseCallback(&CloudGrabberVisualisation::mouseCallback, *this);
      m_cloudViewer->registerKeyboardCallback(&CloudGrabberVisualisation::keyboardCallback, *this);
      m_cloudViewer->setCameraFieldOfView(1.02259994f);

      m_grabber->open();

      bool cloudInit = false;
      while (!m_cloudViewer->wasStopped())
      {
        m_cloudViewer->spinOnce();

        ICloudGrabber::Cloud::ConstPtr cloud = m_grabber->getCloud();
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

  private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_cloudViewer;
    Yukari::CloudCapture::ICloudGrabber_sptr m_grabber;
  };
}
}
