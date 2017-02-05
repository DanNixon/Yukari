/** @file */

#pragma once

#include "BaseMathType.h"

#include <cmath>

namespace Yukari
{
namespace Maths
{
  template <size_t SIZE> class BaseVectorType : public BaseMathType<SIZE>
  {
  public:
    float length() const
    {
      return std::sqrt(length2());
    }

    float length2() const
    {
      float len = 0;
      for (size_t i = 0; i < SIZE; i++)
        len += std::pow(this->m_values[i], 2);
      return len;
    }

    float distance(const BaseVectorType &other) const
    {
      return std::sqrt(distance2(other));
    }

    float distance2(const BaseVectorType &other) const
    {
      BaseVectorType delta(*this);
      delta -= other;
      return delta.length2();
    }

    void normalise()
    {
      float l = length();

      if (l != 0.0f)
      {
        l = 1.0f / l;

        for (size_t i = 0; i < SIZE; i++)
          this->m_values[i] *= l;
      }
    }

    BaseVectorType &operator+=(const BaseVectorType &rhs)
    {
      for (size_t i = 0; i < 4; i++)
        this->m_values[i] += rhs.m_values[i];

      return *this;
    }

    BaseVectorType &operator-=(const BaseVectorType &rhs)
    {
      for (size_t i = 0; i < 4; i++)
        this->m_values[i] -= rhs.m_values[i];

      return *this;
    }

    BaseVectorType &operator*=(const BaseVectorType &rhs)
    {
      for (size_t i = 0; i < 4; i++)
        this->m_values[i] *= rhs.m_values[i];

      return *this;
    }

    BaseVectorType &operator*=(float rhs)
    {
      for (size_t i = 0; i < 4; i++)
        this->m_values[i] *= rhs;

      return *this;
    }

    BaseVectorType &operator/=(const BaseVectorType &rhs)
    {
      for (size_t i = 0; i < 4; i++)
        this->m_values[i] /= rhs.m_values[i];

      return *this;
    }

    BaseVectorType &operator/=(float rhs)
    {
      for (size_t i = 0; i < 4; i++)
        this->m_values[i] /= rhs;

      return *this;
    }
  };
}
}
