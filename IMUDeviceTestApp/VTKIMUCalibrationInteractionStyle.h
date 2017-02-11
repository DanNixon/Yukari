#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IGrabber.h>

class VTKIMUCalibrationInteractionStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static VTKIMUCalibrationInteractionStyle *New();
  vtkTypeMacro(VTKIMUCalibrationInteractionStyle, vtkInteractorStyleTrackballCamera);

  virtual void OnKeyPress();

  inline void setGrabber(Yukari::IMU::IGrabber *grabber)
  {
    m_grabber = grabber;
  }

private:
  Yukari::IMU::IGrabber *m_grabber;
  Yukari::Common::LoggingService::Logger m_logger;
};
