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
#include <YukariIMU/IGrabber.h>

class VTKIMUActorCallback : public vtkCommand
{
public:
  static VTKIMUActorCallback *New();

  virtual void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                       void *vtkNotUsed(callData));

  inline void setGrabber(Yukari::IMU::IGrabber *grabber)
  {
    m_grabber = grabber;
  }

  inline void setActor(vtkSmartPointer<vtkActor> actor)
  {
    m_actor = actor;
  }

private:
  Yukari::IMU::IGrabber *m_grabber;
  vtkSmartPointer<vtkActor> m_actor;
  Yukari::Common::LoggingService::Logger m_logger;
};
