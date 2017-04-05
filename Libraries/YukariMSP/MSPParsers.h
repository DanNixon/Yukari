/** @file */

#pragma once

#include <YukariMaths/Quaternion.h>

#include "MSPClient.h"

namespace Yukari
{
namespace MSP
{
  class MSPParsers
  {
  public:
    static int16_t Read16(MSPClient::Payload::const_iterator it);

    static bool ParseRawIMUPayload(const MSPClient::Payload &payload, int16_t *gyro, int16_t *acc,
                                   int16_t *mag);

    static bool ParseAttitudePayload(const MSPClient::Payload &payload, float *att);

    static bool ParseQuaternion(const MSPClient::Payload &payload, Maths::Quaternion &quat);
  };
}
}
