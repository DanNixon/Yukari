#pragma once

#include <Arduino.h>
#include <functional>

class MSP
{
public:
  static const uint8_t BUFFER_LEN = 20;

  static const uint8_t COMMAND_BYTE_1 = '$';
  static const uint8_t COMMAND_BYTE_2 = 'M';

  enum class Direction : uint8_t
  {
    TO_DEVICE = '>',
    FROM_DEVICE = '<'
  };

  typedef std::function<void(Direction dir, uint8_t cmd, uint8_t *buff, uint8_t len)>
      OnMessageFunc;

public:
  MSP(Stream &stream);
  virtual ~MSP();

  void loop();

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
