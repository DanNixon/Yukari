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
    DummyCloudGrabber();

    virtual void open() override;
    virtual void close() override;
    virtual bool isOpen() const override;

    virtual ICloudGrabber::Cloud::ConstPtr grabCloud() override;

  private:
    bool m_open;
  };
}
}
