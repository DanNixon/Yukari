/** @file */

#include "MSPClient.h"

#include <iostream>

using namespace Yukari::Common;

namespace Yukari
{
namespace MSP
{
  void MSPClient::BuildCommandPayload(Payload &payload, MSPCommand command, const Payload &data)
  {
    /* Build payload */
    payload.push_back('$');
    payload.push_back('M');
    payload.push_back('<');
    payload.push_back((uint8_t)data.size());
    payload.push_back((uint8_t)command);
    payload.insert(payload.end(), data.cbegin(), data.cend());

    /* Generate checksum */
    uint8_t checksum = 0;
    for (auto it = payload.begin() + 3; it != payload.end(); ++it)
      checksum ^= *it;

    payload.push_back(checksum);
  }

  bool MSPClient::ParseResponsePayload(const Payload &payload, MSPCommand &command, Payload &data)
  {
    command = MSPCommand::NOP;

    /* Test checksum */
    uint8_t checksum = 0;
    for (auto it = payload.begin() + 3; it != payload.end() - 1; ++it)
      checksum ^= *it;

    if (checksum != payload.back())
    {
      LoggingService::Instance()
          .getLogger("MSPClient")
          ->error("Checksum mismatch: got {}, expected {}", payload.back(), checksum);
      return false;
    }

    /* Parse data */
    command = (MSPCommand)payload[4];
    data.reserve(payload[3]);
    data.insert(data.begin(), payload.begin() + 5, payload.end() - 1);

    return true;
  }

  MSPClient::MSPClient(serial::Serial &port)
      : m_logger(LoggingService::Instance().getLogger("MSPClient"))
      , m_port(port)
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
    m_logger->trace("MSP TX payload: {}", txPayload);
    bytesTransfered = m_port.write(txPayload);
    if (bytesTransfered != txPayload.size())
      return false;

    /* Receive payload */
    Payload rxPayload;

    bytesTransfered = m_port.read(rxPayload, 5);
    if (bytesTransfered != 5)
    {
      m_logger->error("Incorrect number of bytes received: {} (expected 5)", bytesTransfered);
      return false;
    }

    size_t numBytesToRx = rxPayload[3] + 1;
    bytesTransfered = m_port.read(rxPayload, numBytesToRx);
    if (bytesTransfered != numBytesToRx)
    {
      m_logger->error("Incorrect number of bytes received: {} (expected {})", bytesTransfered,
                      numBytesToRx);
      return false;
    }

    /* Parse response payload */
    payload.clear();
    return ParseResponsePayload(rxPayload, command, payload);
  }
}
}

std::ostream &operator<<(std::ostream &str, const Yukari::MSP::MSPClient::Payload &payload)
{
  str << "Payload[" << std::hex;

  for (auto it = payload.cbegin(); it != payload.cend(); ++it)
  {
    if (it != payload.cbegin())
      str << ',';

    str << std::hex << (int)*it;
  }

  str << "]" << std::dec;

  return str;
}
