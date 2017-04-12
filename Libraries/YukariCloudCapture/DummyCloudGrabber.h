/** @file */

#pragma once

#include "ICloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE> class DummyCloudGrabber : public ICloudGrabber<POINT_TYPE>
  {
  public:
    DummyCloudGrabber(size_t width = 10, size_t height = 10)
        : m_width(width)
        , m_height(height)
    {
    }

    virtual CloudConstPtr grabCloud() override
    {
      auto retVal = new Cloud();

      retVal->width = m_width;
      retVal->height = m_height;
      retVal->is_dense = false;
      retVal->points.resize(retVal->width * retVal->height);

      for (size_t i = 0; i < retVal->points.size(); ++i)
      {
        retVal->points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
        retVal->points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
        retVal->points[i].z = 10 * rand() / (RAND_MAX + 1.0f);

        retVal->points[i].rgba = 0xFFFFFFFF;
      }

      return CloudConstPtr(retVal);
    }

  protected:
    size_t m_width;
    size_t m_height;
  };
}
}
