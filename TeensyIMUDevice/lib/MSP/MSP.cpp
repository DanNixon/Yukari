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
      // Buffer contains ['$', 'M', dir, len, cmd]
      m_bufferCommandLength = 4 + m_buffer[3];
    }
    else if (m_bufferPos == m_bufferCommandLength)
    {
      bool checksumPass = true; // TODO

      if (checksumPass && m_onMessage)
        m_onMessage((Direction)m_buffer[2], (Command)m_buffer[4], m_buffer + 5, m_buffer[3]);

      resetBuffer();
    }
  }
}

void MSP::resetBuffer()
{
  m_bufferPos = 0;
  m_bufferCommandLength = 0;
  memset(m_buffer, '\0', BUFFER_LEN);
}
