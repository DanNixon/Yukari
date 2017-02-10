/** @file */

#include "MSPGrabber.h"

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Vector3.h>

using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{
  MSPGrabber::MSPGrabber(const std::string &port, unsigned int baud)
      : m_port(port, baud, serial::Timeout::simpleTimeout(1000))
      , m_client(m_port)
  {
  }

  MSPGrabber::~MSPGrabber()
  {
    close();
  }

  void MSPGrabber::open()
  {
    if (!m_port.isOpen())
      m_port.open();

    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  void MSPGrabber::close()
  {
    if (m_port.isOpen())
      m_port.close();
  }

  bool MSPGrabber::isOpen() const
  {
    return m_port.isOpen();
  }

  bool MSPGrabber::calibrateAccelerometer()
  {
    /* TODO */
    return false;
  }

  bool MSPGrabber::calibrateMagnetometer()
  {
    /* TODO */
    return false;
  }
}
}
