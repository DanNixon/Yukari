/** @file */

#pragma once

#include <cstddef>
#include <ostream>
#include <stdexcept>

#include "Vector4.h"

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

    bool operator==(const Matrix &other) const;
    bool operator!=(const Matrix &other) const;

    void toZero();

    Vector4 row(size_t idx) const;
    Vector4 column(size_t idx) const;

    void setRow(size_t idx, const Vector4 &row);
    void setColumn(size_t idx, const Vector4 &column);

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

    friend std::ostream &operator<<(std::ostream &s, const Matrix &m);

  protected:
    float m_values[NUM_ELEMENTS];
  };
}
}
