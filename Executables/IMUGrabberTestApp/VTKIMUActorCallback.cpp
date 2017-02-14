#include "VTKIMUActorCallback.h"

using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMUGrabberTestApp
{
VTKIMUActorCallback *VTKIMUActorCallback::New()
{
  VTKIMUActorCallback *o = new VTKIMUActorCallback;
  o->m_logger = LoggingService::GetLogger("VTKIMUActorCallback");
  o->m_logger->info("Blue is front of device, red is back of device");
  return o;
}

void VTKIMUActorCallback::Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                                  void *vtkNotUsed(callData))
{
  auto frame = m_grabber->grabFrame();
  if (!frame)
  {
    m_logger->error("Failed to grab frame!");
    return;
  }

  m_logger->info("Frame: {}", (*frame));

  m_actor->SetOrientation(0, 0, 0);

  auto q = frame->orientation();
  float angle = q.getAngle(DEGREES);
  auto axis = q.getAxis();
  m_logger->info("Angle={}, Axis={}", angle, axis);

  m_actor->RotateWXYZ(angle, axis.z(), axis.x(), axis.y());

  vtkRenderWindowInteractor *rendererInteractor = vtkRenderWindowInteractor::SafeDownCast(caller);
  rendererInteractor->GetRenderWindow()->Render();
}
}
}
