#pragma once

#include <vtkActor.h>
#include <vtkAxes.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IIMUGrabber.h>

namespace Yukari
{
namespace IMUGrabberTestApp
{
  class VTKIMUActorCallback : public vtkCommand
  {
  public:
    static VTKIMUActorCallback *New();

    virtual void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                         void *vtkNotUsed(callData));

    inline void setGrabber(Yukari::IMU::IIMUGrabber_sptr grabber)
    {
      m_grabber = grabber;
    }

    inline void setActor(vtkSmartPointer<vtkActor> actor)
    {
      m_actor = actor;
    }

  private:
    Yukari::IMU::IIMUGrabber_sptr m_grabber;
    vtkSmartPointer<vtkActor> m_actor;
    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
