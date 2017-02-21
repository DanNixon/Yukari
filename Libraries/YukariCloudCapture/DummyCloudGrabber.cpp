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
    auto retVal = new ICloudGrabber::Cloud();

    retVal->width = 250;
    retVal->height = 250;
    retVal->is_dense = false;
    retVal->points.resize(retVal->width * retVal->height);

    for (size_t i = 0; i < retVal->points.size(); ++i)
    {
      retVal->points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
      retVal->points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
      retVal->points[i].z = 10 * rand() / (RAND_MAX + 1.0f);

      retVal->points[i].r = 1.0f;
      retVal->points[i].g = 1.0f;
      retVal->points[i].b = 1.0f;
    }

    return ICloudGrabber::Cloud::ConstPtr(retVal);
  }
}
}
