/** @file */

#pragma once

#include "BaseMathType.h"

namespace Yukari
{
namespace Maths
{
  class Quat : public BaseMathType
  {
  public:
    Quat(float i = 0.0f, float j = 0.0f, float k = 0.0f, float w = 1.0f);
    virtual ~Quat();

    inline float i() const
    {
      return m_x;
    }

    inline float j() const
    {
      return m_y;
    }

    inline float k() const
    {
      return m_z;
    }
  };
}
}
