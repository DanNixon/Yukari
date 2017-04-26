/** @file */

#include "FIleIMUGrabber.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  FIleIMUGrabber::FIleIMUGrabber()
  {
  }

  void FIleIMUGrabber::open()
  {
    m_open = true;
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  void FIleIMUGrabber::close()
  {
    m_open = false;
  }

  bool FIleIMUGrabber::isOpen() const
  {
    return m_open;
  }

  IMUFrame::Ptr FIleIMUGrabber::grabFrame()
  {
    /* Calculate timestep */
    auto timeNow = std::chrono::high_resolution_clock::now();
    auto frameDuration = timeNow - m_lastFrameTime;
    m_lastFrameTime = timeNow;

    auto retVal = std::make_shared<IMUFrame>(frameDuration);

    /* TODO */

    return retVal;
  }
}
}
