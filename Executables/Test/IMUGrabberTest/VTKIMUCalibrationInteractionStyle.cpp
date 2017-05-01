#include "VTKIMUCalibrationInteractionStyle.h"

#include <vtkRenderWindowInteractor.h>

using namespace Yukari::Common;
using namespace Yukari::IMU;

namespace Yukari
{
namespace IMUGrabberTest
{
  VTKIMUCalibrationInteractionStyle *VTKIMUCalibrationInteractionStyle::New()
  {
    VTKIMUCalibrationInteractionStyle *o = new VTKIMUCalibrationInteractionStyle();
    o->m_logger = LoggingService::Instance().getLogger("VTKIMUCalibrationInteractionStyle");
    o->m_logger->info("Press A for accelerometer calibration.");
    o->m_logger->info("Press M for magnetometer calibration.");
    o->m_logger->info("Press O to reset device integrators.");
    return o;
  }

  void VTKIMUCalibrationInteractionStyle::OnKeyPress()
  {
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    if (key == "a")
    {
      m_logger->info("Acelerometer calibration requested.");

      if (m_mspGrabber)
      {
        bool result = m_mspGrabber->calibrateAccelerometer();
        if (result)
          m_logger->info("Accelerometer calibration complete.");
        else
          m_logger->error("Accelerometer calibration failed!");
      }
    }

    if (key == "m")
    {
      m_logger->info("Magnetometer calibration requested.");

      if (m_mspGrabber)
      {
        bool result = m_mspGrabber->calibrateAccelerometer();
        if (result)
          m_logger->info("Magnetometer calibration complete.");
        else
          m_logger->error("Magnetometer calibration failed!");
      }
    }

    if (key == "o")
    {
      m_logger->info("Device integrator reset requested.");

      if (m_stm32Grabber)
        m_stm32Grabber->resetDisplacement();
    }

    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }
}
}
