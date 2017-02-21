/** @file */

#include "DummyCloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  DummyCloudGrabber::DummyCloudGrabber()
      : m_open(false)
  {
  }

  void DummyCloudGrabber::open()
  {
    m_open = true;
  }

  void DummyCloudGrabber::close()
  {
    m_open = false;
  }

  bool DummyCloudGrabber::isOpen() const
  {
    return m_open;
  }

  ICloudGrabber::Cloud::ConstPtr DummyCloudGrabber::grabCloud()
  {
    return ICloudGrabber::Cloud::ConstPtr(new ICloudGrabber::Cloud());
  }
}
}
