/** @file */

#pragma once

#include <serial/serial.h>
#include <vector>

namespace Yukari
{
namespace IMU
{
  class MSPClient
  {
  public:
    typedef std::vector<uint8_t> Payload;

    enum MSPCommand
    {
      NOP = 0,
      IDENT = 100,
      STATUS = 101,
      RAW_IMU = 102,
      SERVO = 103,
      MOTOR = 104,
      RC = 105,
      RAW_GPS = 106,
      COMP_GPS = 107,
      ATTITUDE = 108,
      ALTITUDE = 109,
      ANALOG = 110,
      RC_TUNING = 111,
      PID = 112,
      BOX = 113,
      MISC = 114,
      MOTOR_PINS = 115,
      BOXNAMES = 116,
      PIDNAMES = 117,
      WP = 118,
      BOXIDS = 119,
      RC_RAW_IMU = 121,
      SET_RAW_RC = 200,
      SET_RAW_GPS = 201,
      SET_PID = 202,
      SET_BOX = 203,
      SET_RC_TUNING = 204,
      ACC_CALIBRATION = 205,
      MAG_CALIBRATION = 206,
      SET_MISC = 207,
      RESET_CONF = 208,
      SET_WP = 209,
      SWITCH_RC_SERIAL = 210,
      IS_SERIAL = 211,
      DEBUG = 254
    };

  public:
    static void BuildCommandPayload(Payload &payload, MSPCommand command, const Payload &data);
    static bool ParseResponsePayload(const Payload &payload, MSPCommand &command, Payload &data);

    static int16_t Read16(Payload::const_iterator it);

    static bool ParseRawIMUPayload(const Payload &payload, float *gyro, float *acc, float *mag);

  public:
    MSPClient(serial::Serial &port);

    bool requestData(MSPCommand command, Payload &payload);

  private:
    serial::Serial &m_port;
  };
}
}
