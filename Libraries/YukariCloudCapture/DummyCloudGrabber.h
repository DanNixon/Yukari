/** @file */

#pragma once

#include "ICloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  class DummyCloudGrabber : public ICloudGrabber
  {
  public:
    DummyCloudGrabber(size_t width = 25, size_t height = 25);

    virtual void open() override;
    virtual void close() override;
    virtual bool isOpen() const override;

    virtual ICloudGrabber::Cloud::ConstPtr grabCloud() override;

  private:
    const size_t m_width;
    const size_t m_height;

    bool m_open;
  };
}
}
