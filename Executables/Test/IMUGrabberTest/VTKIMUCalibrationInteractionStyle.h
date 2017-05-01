#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMSPGrabber.h>
#include <YukariIMU/STM32IMUDevice.h>

namespace Yukari
{
namespace IMUGrabberTest
{
  class VTKIMUCalibrationInteractionStyle : public vtkInteractorStyleTrackballCamera
  {
  public:
    static VTKIMUCalibrationInteractionStyle *New();
    vtkTypeMacro(VTKIMUCalibrationInteractionStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnKeyPress();

    inline void setGrabber(Yukari::IMU::IIMUGrabber::Ptr grabber)
    {
      m_mspGrabber = std::dynamic_pointer_cast<Yukari::IMU::IMSPGrabber>(grabber);
      m_stm32Grabber = std::dynamic_pointer_cast<Yukari::IMU::STM32IMUDevice>(grabber);
    }

  private:
    Yukari::IMU::IMSPGrabber::Ptr m_mspGrabber;
    Yukari::IMU::STM32IMUDevice::Ptr m_stm32Grabber;
    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
