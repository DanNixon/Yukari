/** @file */

#include "MSPParsers.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace MSP
{
  int16_t MSPParsers::Read16(MSPClient::Payload::const_iterator it)
  {
    union {
      int16_t v;
      uint8_t b[2];
    } s;

    s.b[0] = *(it++);
    s.b[1] = *it;

    return s.v;
  }

  bool MSPParsers::ParseRawIMUPayload(const MSPClient::Payload &payload, int16_t *gyro,
                                      int16_t *acc, int16_t *mag)
  {
    if (payload.size() != 18)
      return false;

    auto data = payload.begin();

    size_t i;

    for (i = 0; i < 3; i++)
    {
      acc[i] = Read16(data);
      data += 2;
    }

    for (i = 0; i < 3; i++)
    {
      gyro[i] = Read16(data);
      data += 2;
    }

    for (i = 0; i < 3; i++)
    {
      mag[i] = Read16(data);
      data += 2;
    }

    return true;
  }

  bool MSPParsers::ParseAttitudePayload(const MSPClient::Payload &payload, float *att)
  {
    if (payload.size() != 6)
      return false;

    auto data = payload.begin();

    /* X axis */
    att[0] = Read16(data) / 10.0f;
    data += 2;

    /* Y axis */
    att[1] = Read16(data) / 10.0f;
    data += 2;

    /* Heading */
    att[2] = Read16(data);

    return true;
  }

  bool MSPParsers::ParseQuaternion(const MSPClient::Payload &payload, Eigen::Quaternionf &quat)
  {
    if (payload.size() != 8)
      return false;

    static const float F = 16384.0f;

    float q[4];
    q[0] = ((payload[0] << 8) | payload[1]) / F;
    q[1] = ((payload[2] << 8) | payload[3]) / F;
    q[2] = ((payload[4] << 8) | payload[5]) / F;
    q[3] = ((payload[6] << 8) | payload[7]) / F;

    for (size_t i = 0; i < 4; i++)
    {
      if (q[i] >= 2)
        q[i] = -4 + q[i];
    }

    quat = Eigen::Quaternionf(q[0], -q[1], q[3], q[2]);

    return true;
  }

  bool MSPParsers::ParseVector3(const MSPClient::Payload &payload, Eigen::Vector3f &vec)
  {
    // TODO
    vec = Eigen::Vector3f::Zero();
    return false;
  }
}
}
