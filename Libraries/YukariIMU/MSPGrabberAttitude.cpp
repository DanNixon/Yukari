/** @file */

#include "MSPGrabberAttitude.h"

#include <boost/qvm/all.hpp>

#include <YukariMSP/MSPParsers.h>
#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

using namespace boost::qvm;
using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::MSP;

namespace Yukari
{
namespace IMU
{
  MSPGrabberAttitude::MSPGrabberAttitude(const std::string &port, unsigned int baud)
      : MSPGrabber(port, baud)
      , m_logger(LoggingService::GetLogger("MSPGrabberIMU"))
  {
    m_mspPayload.reserve(6);
  }

  MSPGrabberAttitude::~MSPGrabberAttitude()
  {
  }

  IMUFrame_sptr MSPGrabberAttitude::grabFrame()
  {
    m_mspPayload.clear();

    /* MSP data */
    bool ok = m_client.requestData(MSPClient::ATTITUDE, m_mspPayload) &&
              MSPParsers::ParseAttitudePayload(m_mspPayload, m_attitude);

    if (!ok)
    {
      m_logger->error("Failed to get all MSP data");
      return nullptr;
    }

    /* Calculate timestep */
    auto timeNow = std::chrono::high_resolution_clock::now();
    auto frameDuration = timeNow - m_lastFrameTime;
    m_lastFrameTime = timeNow;

    auto retVal = std::make_shared<IMUFrame>(frameDuration);

    /* Set orientation from attitude data */
    Quaternion x, y, z;
    rotate_x(x, -m_attitude[1] * DEG_TO_RAD);
    rotate_y(y, -m_attitude[0] * DEG_TO_RAD);
    rotate_z(z, -m_attitude[2] * DEG_TO_RAD);
    retVal->orientation() = x * y * z;

    return retVal;
  }
}
}
