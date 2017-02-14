#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/MSPGrabber.h>

namespace Yukari
{
namespace IMUGrabberTestApp
{
class VTKIMUCalibrationInteractionStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static VTKIMUCalibrationInteractionStyle *New();
  vtkTypeMacro(VTKIMUCalibrationInteractionStyle, vtkInteractorStyleTrackballCamera);

  virtual void OnKeyPress();

  inline void setGrabber(Yukari::IMU::IIMUGrabber_sptr grabber)
  {
    m_grabber = std::dynamic_pointer_cast<Yukari::IMU::MSPGrabber>(grabber);
  }

private:
  Yukari::IMU::MSPGrabber_sptr m_grabber;
  Yukari::Common::LoggingService::Logger m_logger;
};
}
}
