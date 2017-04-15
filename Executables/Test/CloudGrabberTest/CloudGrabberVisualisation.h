#pragma once

#include <memory>

#include <pcl/visualization/pcl_visualizer.h>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariProcessing/SpatialOperations.h>

namespace Yukari
{
namespace CloudGrabberTest
{
  template <typename POINT_TYPE> class CloudGrabberVisualisation
  {
  public:
    typedef typename CloudCapture::ICloudGrabber<POINT_TYPE>::Ptr GrabberPtr;

  public:
    CloudGrabberVisualisation(GrabberPtr grabber, IMU::IIMUGrabber_sptr imu)
        : m_cloudViewer(new pcl::visualization::PCLVisualizer("Cloud grabber visualisation"))
        , m_grabber(grabber)
        , m_imu(imu)
    {
    }

    void run()
    {
      m_cloudViewer->setCameraFieldOfView(1.02259994f);

      m_grabber->open();
      if (m_imu)
        m_imu->open();

      bool cloudInit = false;
      while (!m_cloudViewer->wasStopped())
      {
        m_cloudViewer->spinOnce();

        auto cloud = m_grabber->grabCloud();

        if (cloud)
        {
          if (m_imu)
          {
            auto imuFrame = m_imu->grabFrame();
            if (imuFrame)
              pcl::transformPointCloud(*cloud, *cloud, imuFrame->position().toEigen(),
                                       Processing::SpatialOperations::RotateQuaternionForCloud(
                                           imuFrame->orientation().toEigen()));
          }

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
          }
        }
      }

      m_grabber->close();
      if (m_imu)
        m_imu->close();
    }

  private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_cloudViewer;

    GrabberPtr m_grabber;
    IMU::IIMUGrabber_sptr m_imu;
  };
}
}
