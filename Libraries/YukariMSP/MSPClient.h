/** @file */

#pragma once

#include <serial/serial.h>
#include <vector>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace MSP
{
  /**
   * @class MSPClient
   * @brief Client for sending and receiving data over the MultiWii Serial protocol.
   *
   * The MSP client was coded with reference to the following:
   *  - http://www.multiwii.com/forum/viewtopic.php?f=8&t=1516
   *  - http://www.multiwii.com/forum/viewtopic.php?f=8&t=4518#
   *  - https://github.com/alduxvm/pyMultiWii
   */
  class MSPClient
  {
  public:
    typedef std::vector<uint8_t> Payload;

    /**
     * @brief Commands for custom MSP interface
     *
     * Some are from "original" MSP, some are custom (prefixed with "Y_").
     */
    enum MSPCommand
    {
      NOP = 0,

      IDENT = 100,
      STATUS = 101,
      RAW_IMU = 102,
      RAW_GPS = 106,
      COMP_GPS = 107,
      ATTITUDE = 108,
      ALTITUDE = 109,
      RC_RAW_IMU = 121,
      SET_RAW_GPS = 201,
      ACC_CALIBRATION = 205,
      MAG_CALIBRATION = 206,
      RESET_CONF = 208,
      MSP_DEBUG = 254,

      Y_RAW_IMU = 150,
      Y_LINEAR_ACC_REAL = 151,
      Y_LINEAR_ACC_WORLD = 152,
      Y_ORIENTATION = 153,
      Y_DISPLACEMENT = 154,
      Y_RESET_DISPLACEMENT = 155
    };

  public:
    static void BuildCommandPayload(Payload &payload, MSPCommand command, const Payload &data);
    static bool ParseResponsePayload(const Payload &payload, MSPCommand &command, Payload &data);

  public:
    MSPClient(serial::Serial &port);

    bool requestData(MSPCommand command, Payload &payload);

  private:
    Common::LoggingService::Logger m_logger;
    serial::Serial &m_port;
  };
}
}

std::ostream &operator<<(std::ostream &str, const Yukari::MSP::MSPClient::Payload &payload);
