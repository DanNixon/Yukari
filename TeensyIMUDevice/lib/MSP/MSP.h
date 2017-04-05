#pragma once

#include <Arduino.h>
#include <functional>

class MSP
{
public:
  static const uint8_t BUFFER_LEN = 20;

  static const uint8_t COMMAND_BYTE_1 = '$';
  static const uint8_t COMMAND_BYTE_2 = 'M';

  enum class Command : uint8_t
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
    Y_ORIENTATION = 153
  };

  enum class Direction : uint8_t
  {
    TO_DEVICE = '<',
    FROM_DEVICE = '>'
  };

  typedef std::function<void(Direction dir, Command cmd, uint8_t *buff, uint8_t len)> OnMessageFunc;

public:
  MSP(Stream &stream);
  virtual ~MSP();

  void loop();
  void sendPacket(Command cmd, uint8_t *buff, uint8_t len);

  inline void setOnMessage(OnMessageFunc f)
  {
    m_onMessage = f;
  }

protected:
  void resetBuffer();

protected:
  Stream &m_stream;

  OnMessageFunc m_onMessage;

  uint8_t m_bufferPos;
  uint8_t m_bufferCommandLength;
  uint8_t m_buffer[BUFFER_LEN];
};
