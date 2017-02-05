/** @file */

#include "MSPGrabber.h"

namespace Yukari
{
namespace IMU
{
  MSPGrabber::MSPGrabber(const std::string &port, int baud)
      : m_port(port, baud, serial::Timeout::simpleTimeout(1000))
  {
  }

  MSPGrabber::~MSPGrabber()
  {
    close();
  }

  void MSPGrabber::open()
  {
    m_port.open();
  }

  void MSPGrabber::close()
  {
    m_port.close();
  }

  bool MSPGrabber::isOpen() const
  {
    return m_port.isOpen();
  }

  IMUFrame_sptr MSPGrabber::grabFrame()
  {
    /* TODO */
    return nullptr;
  }
}
}
