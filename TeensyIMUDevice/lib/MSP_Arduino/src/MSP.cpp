/*
  MSP - Library for MultiWii Serial Protocol
  Created by Filip Prochazka (jacktech24), 2015.
  Released under GNUGPLv3 license.
*/

#include "MSP.h"
#include "Arduino.h"
#include "MSP_Data.h"
#include "MSP_Processors.h"

MSP::MSP(Stream &port)
    : m_port(port)
{
}

void MSP::begin()
{
  MSP::sendMsgRequest(MSP_MSG_IDENT);
}

void MSP::loop()
{
  while (m_port.available())
  {
    uint8_t inVal = (uint8_t)m_port.read();
    if (inVal == MSP_PREAMBLE_PART1)
    {
      // possible start of message
      inVal = (uint8_t)m_port.read();
      if (inVal == MSP_PREAMBLE_PART2)
      {
        // confirmation with second byte
        MSP::processInputMsg();
      }
    }
  }
}

void MSP::processInputMsg()
{
  uint8_t direction = (uint8_t)m_port.read();
  uint8_t payloadLength = (uint8_t)m_port.read();
  uint8_t msgId = (uint8_t)m_port.read();

  uint8_t *payload = MSP::readMsgDataPart(payloadLength);

  if (direction == MSP_DIRECTION_IN)
  {
    switch (msgId)
    {
    case MSP_MSG_IDENT:
      MSP_Processors::process_Ident(payloadLength, payload, m_callbacks.onIdent);
      break;
    case MSP_MSG_STATUS:
      MSP_Processors::process_Status(payloadLength, payload, m_callbacks.onStatus);
      break;
    default:
      // ignore, unknown or not important message
      break;
    }
  }
}

uint8_t *MSP::readMsgDataPart(uint8_t payloadLength)
{
  m_payload[payloadLength] = {};
  for (int i = 0; i < payloadLength; i++)
  {
    m_payload[i] = (uint8_t)m_port.read();
  }
  return m_payload;
}

void MSP::sendMsgRequest(MSP_Messages messages)
{
}
