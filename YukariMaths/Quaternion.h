/** @file */

#pragma once

#include "BaseMathType.h"

namespace Yukari
{
namespace Maths
{
  class Quaternion : public BaseMathType
  {
  public:
    Quaternion(float i = 0.0f, float j = 0.0f, float k = 0.0f, float w = 1.0f);

    inline float i() const
    {
      return m_x;
    }

    inline float &i()
    {
      return m_x;
    }

    inline float j() const
    {
      return m_y;
    }

    inline float &j()
    {
      return m_y;
    }

    inline float k() const
    {
      return m_z;
    }

    inline float &k()
    {
      return m_z;
    }

    /* TODO */
  };
}
}
