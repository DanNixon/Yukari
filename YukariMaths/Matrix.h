/** @file */

#pragma once

#include <cstddef>
#include <stdexcept>

#include "Vector.h"

namespace Yukari
{
namespace Maths
{
  class Matrix
  {
  public:
    static const size_t NUM_ELEMENTS = 16;

  public:
    Matrix();
    virtual ~Matrix();

    bool operator==(const Matrix &other) const;
    bool operator!=(const Matrix &other) const;

    void toZero();

    Vector column(size_t idx) const;
    Vector row(size_t idx) const;

    inline float operator[](size_t idx) const
    {
      if (idx > NUM_ELEMENTS)
        return 0.0f;

      return m_values[idx];
    }

    inline float &operator[](size_t idx)
    {
      if (idx > NUM_ELEMENTS)
        throw std::runtime_error("Matrix index out of range!");

      return m_values[idx];
    }

  protected:
    float m_values[NUM_ELEMENTS];
  };
}
}
