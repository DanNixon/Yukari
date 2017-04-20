#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/MSPGrabber.h>

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
      m_grabber = std::dynamic_pointer_cast<Yukari::IMU::MSPGrabber>(grabber);
    }

  private:
    Yukari::IMU::MSPGrabber::Ptr m_grabber;
    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
