/** @file */

#pragma once

#include "BaseMathType.h"

#include <boost/qvm/mat_traits.hpp>

#include "Vector4.h"

namespace Yukari
{
namespace Maths
{
  class Matrix4 : public BaseMathType<16>
  {
  public:
    Matrix4();

    friend std::ostream &operator<<(std::ostream &s, const Matrix4 &m);

    friend boost::qvm::mat_traits<Matrix4>;
  };
}
}

namespace boost
{
namespace qvm
{
  template <> struct mat_traits<Yukari::Maths::Matrix4>
  {
    static int const rows = 4;
    static int const cols = 4;
    typedef float scalar_type;

    template <int R, int C> static inline scalar_type &write_element(Yukari::Maths::Matrix4 &m)
    {
      return m.m_values[(C * rows) + R];
    }

    template <int R, int C> static inline scalar_type read_element(Yukari::Maths::Matrix4 const &m)
    {
      return m.m_values[(C * rows) + R];
    }

    static inline scalar_type &write_element_idx(int r, int c, Yukari::Maths::Matrix4 &m)
    {
      return m.m_values[(c * rows) + r];
    }

    static inline scalar_type read_element_idx(int r, int c, Yukari::Maths::Matrix4 const &m)
    {
      return m.m_values[(c * rows) + r];
    }
  };
}
}
