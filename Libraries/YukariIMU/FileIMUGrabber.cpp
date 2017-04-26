/** @file */

#include "FileIMUGrabber.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  FileIMUGrabber::FileIMUGrabber()
  {
  }

  void FileIMUGrabber::open()
  {
    m_open = true;
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  void FileIMUGrabber::close()
  {
    m_open = false;
  }

  bool FileIMUGrabber::isOpen() const
  {
    return m_open;
  }

  IMUFrame::Ptr FileIMUGrabber::grabFrame()
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
