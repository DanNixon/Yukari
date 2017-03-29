#pragma once

#include <Arduino.h>

class MSP
{
public:
  MSP(Stream &stream);
  virtual ~MSP();

  void begin();
  void loop();

protected:
  Stream &m_stream;
};
