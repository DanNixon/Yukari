/** @file */

#include "MSPClient.h"

#include <iostream>

namespace Yukari
{
namespace IMU
{
  void MSPClient::BuildCommandPayload(Payload &payload, MSPCommand command, const Payload &data)
  {
    /* Build payload */
    payload.push_back('$');
    payload.push_back('M');
    payload.push_back('<');
    payload.push_back(data.size());
    payload.push_back(command);
    payload.insert(payload.end(), data.cbegin(), data.cend());

    /* Generate checksum */
    uint8_t checksum = 0;
    for (auto it = payload.begin() + 3; it != payload.end(); ++it)
      checksum ^= *it;

    payload.push_back(checksum);
  }

  bool MSPClient::ParseResponsePayload(const Payload &payload, MSPCommand &command, Payload &data)
  {
    /* Test checksum */
    uint8_t checksum = 0;
    for (auto it = payload.begin() + 3; it != payload.end() - 1; ++it)
      checksum ^= *it;

    if (checksum != payload.back())
      return false;

    /* Parse data */
    command = (MSPCommand)payload[4];
    data.reserve(payload[3]);
    data.insert(data.begin(), payload.begin() + 5, payload.end() - 1);

    return true;
  }

  int16_t MSPClient::Read16(Payload::const_iterator it)
  {
    union {
      int16_t v;
      uint8_t b[2];
    } s;

    s.b[0] = *(it++);
    s.b[1] = *it;

    return s.v;
  }

  bool MSPClient::ParseRawIMUPayload(const Payload &payload, float *gyro, float *acc, float *mag)
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

  MSPClient::MSPClient(serial::Serial &port)
      : m_port(port)
  {
    m_port.setTimeout(serial::Timeout::max(), 1000, 0, 2000, 0);
  }

  bool MSPClient::requestData(MSPCommand command, Payload &payload)
  {
    if (!m_port.isOpen())
      return false;

    size_t bytesTransfered = 0;

    /* Build and send payload */
    Payload txPayload;
    BuildCommandPayload(txPayload, command, payload);
    bytesTransfered = m_port.write(txPayload);
    if (bytesTransfered != txPayload.size())
      return false;

    /* Receive payload */
    Payload rxPayload;

    bytesTransfered = m_port.read(rxPayload, 5);
    if (bytesTransfered != 5)
      return false;

    bytesTransfered = m_port.read(rxPayload, rxPayload[3] + 1);
    if (bytesTransfered != rxPayload[3] + 1)
      return false;

    /* Parse response payload */
    return ParseResponsePayload(rxPayload, command, payload);
  }
}
}
