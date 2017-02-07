/** @file */

#include "MSPGrabberAttitude.h"

namespace Yukari
{
namespace IMU
{

  MSPGrabberAttitude::MSPGrabberAttitude(const std::string &port, unsigned int baud)
      : m_port(port, baud, serial::Timeout::simpleTimeout(1000))
  {
  }

  MSPGrabberAttitude::~MSPGrabberAttitude()
  {
    close();
  }

  void MSPGrabberAttitude::open()
  {
    m_port.open();
  }

  void MSPGrabberAttitude::close()
  {
    m_port.close();
  }

  bool MSPGrabberAttitude::isOpen() const
  {
    return m_port.isOpen();
  }

  IMUFrame_sptr MSPGrabberAttitude::grabFrame()
  {
    /* TODO */
    return nullptr;
  }
}
}
