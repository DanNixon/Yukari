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
    command = (MSPCommand) payload[4];
    data.reserve(payload[3]);
    data.insert(data.begin(), payload.begin() + 5, payload.end() - 1);

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

    /* Build and send payload */
    Payload txPayload;
    BuildCommandPayload(payload, command, {});
    m_port.write(txPayload);

    std::cout << "writedone\n";

    /* Receive payload */
    Payload rxPayload;
    size_t read = 0;

    read = m_port.read(rxPayload, 5);
    if (read != 5)
      return false;

    read = m_port.read(rxPayload, rxPayload[3] + 1);
    if (read != rxPayload[3] + 1)
      return false;

    /* Parse response payload */
    return ParseResponsePayload(payload, command, rxPayload);
  }
}
}
