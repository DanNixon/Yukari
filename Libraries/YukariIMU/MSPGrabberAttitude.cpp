/** @file */

#include "MSPGrabberAttitude.h"

#include <YukariMSP/MSPParsers.h>
#include <YukariMaths/Units.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::MSP;

namespace Yukari
{
namespace IMU
{
  MSPGrabberAttitude::MSPGrabberAttitude(const std::string &port, unsigned int baud)
      : IMSPGrabber(port, baud)
      , m_logger(LoggingService::Instance().getLogger("MSPGrabberIMU"))
  {
    m_mspPayload.reserve(6);
  }

  MSPGrabberAttitude::~MSPGrabberAttitude()
  {
  }

  IMUFrame::Ptr MSPGrabberAttitude::grabFrame()
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
    Eigen::Quaternionf rot =
        Eigen::AngleAxisf(-m_attitude[2] * DEG_TO_RAD, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(m_attitude[1] * DEG_TO_RAD, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(m_attitude[0] * DEG_TO_RAD, Eigen::Vector3f::UnitZ());
    retVal->orientation() = m_transform.orientation() * rot;

    /* Set empty position */
    retVal->position() = Eigen::Vector3f::Zero();

    return retVal;
  }
}
}
