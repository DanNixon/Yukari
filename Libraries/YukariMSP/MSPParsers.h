/** @file */

#pragma once

#include <Eigen/Geometry>

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

    static bool ParseQuaternion(const MSPClient::Payload &payload, Eigen::Quaternionf &quat);
	static bool ParseVector3(const MSPClient::Payload &payload, Eigen::Vector3f &vec);
  };
}
}
