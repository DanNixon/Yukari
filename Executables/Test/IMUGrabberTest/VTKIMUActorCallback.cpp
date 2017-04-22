#include "VTKIMUActorCallback.h"

#include <YukariMaths/Units.h>

using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMUGrabberTest
{
  VTKIMUActorCallback *VTKIMUActorCallback::New()
  {
    VTKIMUActorCallback *o = new VTKIMUActorCallback;
    o->m_logger = LoggingService::Instance().getLogger("VTKIMUActorCallback");
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

    m_logger->trace("Frame: {}", (*frame));

    /* Apply position */
    auto p = frame->position();
    m_actor->SetPosition(p.x(), p.y(), p.z());

    /* Obtain orientation as axis angle */
    Eigen::AngleAxisf rot;
    rot = frame->orientation();
    float angle = rot.angle() * RAD_TO_DEG;
    Eigen::Vector3f axis = rot.axis();
    m_logger->debug("Angle={}, Axis={}", angle, axis);

    /* Apply orientation */
    m_actor->SetOrientation(0, 0, 0);
    m_actor->RotateWXYZ(angle, axis.x(), axis.y(), axis.z());

    /* Render */
    vtkRenderWindowInteractor *rendererInteractor = vtkRenderWindowInteractor::SafeDownCast(caller);
    rendererInteractor->GetRenderWindow()->Render();
  }
}
}
