/** @file */

#pragma once

#include "BaseVectorType.h"

namespace Yukari
{
namespace Maths
{
  class Vector3 : public BaseVectorType<3>
  {
  public:
    Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f);

    inline float x() const
    {
      return m_values[0];
    }

    inline float &x()
    {
      return m_values[0];
    }

    inline float y() const
    {
      return m_values[1];
    }

    inline float &y()
    {
      return m_values[1];
    }

    inline float z() const
    {
      return m_values[2];
    }

    inline float &z()
    {
      return m_values[2];
    }
  };
}
}
