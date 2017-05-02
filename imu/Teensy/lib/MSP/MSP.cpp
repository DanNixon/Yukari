#include "MSP.h"

MSP::MSP(Stream &stream)
    : m_stream(stream)
{
  resetBuffer();
}

MSP::~MSP()
{
}

void MSP::loop()
{
  if (m_stream.available())
  {
    m_buffer[m_bufferPos++] = m_stream.read();

    if (m_bufferPos == 5)
    {
      /* Buffer contains ['$', 'M', dir, len, cmd] */
      m_bufferCommandLength = 5 + m_buffer[3];
    }
    else if (m_bufferPos == m_bufferCommandLength + 1)
    {
      bool checksumPass = true; // TODO

      if (checksumPass && m_onMessage)
        m_onMessage((Direction)m_buffer[2], (Command)m_buffer[4], m_buffer + 5, m_buffer[3]);

      resetBuffer();
    }
  }
}

void MSP::sendPacket(Command cmd, uint8_t *buff, uint8_t len)
{
  uint8_t checksum = 0;

  /* Header */
  m_stream.write(COMMAND_BYTE_1);
  m_stream.write(COMMAND_BYTE_2);

  /* Direction */
  m_stream.write((uint8_t)Direction::FROM_DEVICE);

  /* Payload length */
  m_stream.write(len);
  checksum ^= len;

  /* Command */
  m_stream.write((uint8_t)cmd);
  checksum ^= (uint8_t)cmd;

  /* Payload */
  for (uint8_t i = 0; i < len; i++)
  {
    m_stream.write(buff[i]);
    checksum ^= buff[i];
  }

  /* Checksum */
  m_stream.write(checksum);
}

void MSP::resetBuffer()
{
  m_bufferPos = 0;
  m_bufferCommandLength = BUFFER_LEN + 2;
  memset(m_buffer, '\0', BUFFER_LEN);
}
