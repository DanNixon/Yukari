/** @file */

#pragma once

#include "BaseMathType.h"
#include "Vector4.h"

namespace Yukari
{
namespace Maths
{
  class Matrix : public BaseMathType<16>
  {
  public:
    Matrix();

    Vector4 row(size_t idx) const;
    Vector4 column(size_t idx) const;

    void setRow(size_t idx, const Vector4 &row);
    void setColumn(size_t idx, const Vector4 &column);

    friend std::ostream &operator<<(std::ostream &s, const Matrix &m);
  };
}
}
