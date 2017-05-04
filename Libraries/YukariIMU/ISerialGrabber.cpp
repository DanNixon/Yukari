#include "ISerialGrabber.h"

namespace Yukari
{
namespace IMU
{
  ISerialGrabber::ISerialGrabber(const std::string &port, unsigned int baud)
      : m_defaultTimeout(serial::Timeout::simpleTimeout(1000))
      , m_port(port, baud, m_defaultTimeout)
  {
  }

  ISerialGrabber::~ISerialGrabber()
  {
  }

  void ISerialGrabber::open()
  {
    if (!m_port.isOpen())
    {
      m_port.open();
      m_port.flush();
    }
  }

  void ISerialGrabber::close()
  {
    if (m_port.isOpen())
      m_port.close();
  }

  bool ISerialGrabber::isOpen() const
  {
    return m_port.isOpen();
  }
}
}
