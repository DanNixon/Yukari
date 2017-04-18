/** @file */

#include "TeensyIMUDevice.h"

#include <boost/qvm/all.hpp>

#include <YukariMSP/MSPParsers.h>

using namespace boost::qvm;
using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::MSP;

namespace Yukari
{
namespace IMU
{
  TeensyIMUDevice::TeensyIMUDevice(const std::string &port, unsigned int baud)
      : MSPGrabber(port, baud)
      , m_logger(LoggingService::Instance().getLogger("TeensyIMUDevice"))
  {
    m_mspPayloadQuat.reserve(6);
  }

  TeensyIMUDevice::~TeensyIMUDevice()
  {
  }

  IMUFrame_sptr TeensyIMUDevice::grabFrame()
  {
    m_mspPayloadQuat.clear();

    /* MSP data */
    bool ok = m_client.requestData(MSPClient::Y_ORIENTATION, m_mspPayloadQuat);

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

    /* Set orientation */
    Quaternion q;
    MSPParsers::ParseQuaternion(m_mspPayloadQuat, q);
    retVal->orientation() = q * m_transform.orientation();

    /* Set position */
    Vector3 position;
    // TODO: testing only, position offset needs rotated by device orientation
    retVal->position() = position - m_transform.position();

    return retVal;
  }
}
}
