#pragma once

#include <memory>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/grabber.h>

namespace Yukari
{
namespace CloudGrabberTest
{
  template <typename POINT_TYPE>
  class CloudGrabberVisualisation
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

  public:
    CloudGrabberVisualisation(
      std::shared_ptr<pcl::Grabber> grabber)
      : m_cloudViewer(new pcl::visualization::PCLVisualizer("Cloud grabber visualisation"))
      , m_grabber(grabber)
    {
    }

    void run()
    {
      m_cloudViewer->setCameraFieldOfView(1.02259994f);

      m_grabber->start();

      bool cloudInit = false;
      while (!m_cloudViewer->wasStopped())
      {
        m_cloudViewer->spinOnce();

        if (m_cloud)
        {
          if (!cloudInit)
          {
            m_cloudViewer->setPosition(0, 0);
            m_cloudViewer->setSize(m_cloud->width, m_cloud->height);
            cloudInit = !cloudInit;
          }

          if (!m_cloudViewer->updatePointCloud(m_cloud, "OpenNICloud"))
          {
            m_cloudViewer->addPointCloud(m_cloud, "OpenNICloud");
            m_cloudViewer->resetCameraViewpoint("OpenNICloud");
          }
        }
      }

      m_grabber->stop();
    }

  private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_cloudViewer;
    std::shared_ptr<pcl::Grabber> m_grabber;
    CloudConstPtr m_cloud;
  };
}
}
