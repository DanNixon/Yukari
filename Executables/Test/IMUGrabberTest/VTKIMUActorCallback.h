#pragma once

#include <boost/filesystem.hpp>
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
namespace IMUGrabberTest
{
  class VTKIMUActorCallback : public vtkCommand
  {
  public:
    static VTKIMUActorCallback *New();

    virtual void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                         void *vtkNotUsed(callData));

    inline void setGrabber(Yukari::IMU::IIMUGrabber::Ptr grabber)
    {
      m_grabber = grabber;
    }

    inline void setActor(vtkSmartPointer<vtkActor> actor)
    {
      m_actor = actor;
    }

    inline void setDataFile(const boost::filesystem::path &filename)
    {
      m_dataFilename = filename;
    }

  private:
    Yukari::IMU::IIMUGrabber::Ptr m_grabber;
    vtkSmartPointer<vtkActor> m_actor;
    boost::filesystem::path m_dataFilename;

    Yukari::Common::LoggingService::Logger m_logger;
  };
}
}
