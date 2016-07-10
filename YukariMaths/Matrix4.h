/** @file */

#pragma once

#include <cstddef>

#include "Vector.h"

namespace Yukari
{
namespace Maths
{
  class Matrix4
  {
  public:
    Matrix4();
    virtual ~Matrix4();

    bool operator==(const Matrix4 &other) const;
    bool operator!=(const Matrix4 &other) const;

    void toZero();

    Vector column(size_t idx) const;
    Vector row(size_t idx) const;

  protected:
    float m_values[16];
  };
}
}
