#include "VTKIMUActorCallback.h"

#include <fstream>

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
    o->m_dataFilename = boost::filesystem::path("");
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

    /* Save frame data */
    if (!m_dataFilename.empty())
    {
      m_logger->trace("Saving data to file");

      std::ofstream s;
      s.open(m_dataFilename.string(), std::ofstream::out | std::ofstream::app);

      s << frame->orientation().w() << ',' << frame->orientation().x() << ','
        << frame->orientation().y() << ',' << frame->orientation().z() << ','
        << frame->position().x() << ',' << frame->position().y() << ',' << frame->position().z()
        << '\n';

      s.close();

      m_logger->trace("Data file saved");
    }

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
