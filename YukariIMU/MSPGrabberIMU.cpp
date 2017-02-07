/** @file */

#include "MSPGrabberIMU.h"

namespace Yukari
{
namespace IMU
{
  MSPGrabberIMU::MSPGrabberIMU(const std::string &port, unsigned int baud)
      : m_port(port, baud, serial::Timeout::simpleTimeout(1000))
  {
  }

  MSPGrabberIMU::~MSPGrabberIMU()
  {
    close();
  }

  void MSPGrabberIMU::open()
  {
    m_port.open();
  }

  void MSPGrabberIMU::close()
  {
    m_port.close();
  }

  bool MSPGrabberIMU::isOpen() const
  {
    return m_port.isOpen();
  }

  IMUFrame_sptr MSPGrabberIMU::grabFrame()
  {
    /* TODO */
    return nullptr;
  }
}
}
