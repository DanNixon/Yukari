/** @file */

#include "DummyIMUGrabber.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  DummyIMUGrabber::DummyIMUGrabber()
  {
  }

  void DummyIMUGrabber::open()
  {
    m_open = true;
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  void DummyIMUGrabber::close()
  {
    m_open = false;
  }

  bool DummyIMUGrabber::isOpen() const
  {
    return m_open;
  }

  IMUFrame::Ptr DummyIMUGrabber::grabFrame()
  {
    /* Calculate timestep */
    auto timeNow = std::chrono::high_resolution_clock::now();
    auto frameDuration = timeNow - m_lastFrameTime;
    m_lastFrameTime = timeNow;

    auto retVal = std::make_shared<IMUFrame>(frameDuration);

    /* Random position */
    retVal->position() =
        Eigen::Vector3f(10 * std::rand() / (RAND_MAX + 1.0f), 10 * std::rand() / (RAND_MAX + 1.0f),
                        10 * std::rand() / (RAND_MAX + 1.0f));

    return retVal;
  }
}
}
