/** @file */

#pragma once

#include "BaseMathType.h"

#include <boost/qvm/mat_traits.hpp>

#include "Vector4.h"

namespace Yukari
{
namespace Maths
{
  class Matrix3 : public BaseMathType<9>
  {
  public:
    Matrix3();

    friend std::ostream &operator<<(std::ostream &s, const Matrix3 &m);

    friend boost::qvm::mat_traits<Matrix3>;
  };
}
}

namespace boost
{
namespace qvm
{
  template <> struct mat_traits<Yukari::Maths::Matrix3>
  {
    static int const rows = 3;
    static int const cols = 3;
    typedef float scalar_type;

    template <int R, int C> static inline scalar_type &write_element(Yukari::Maths::Matrix3 &m)
    {
      return m.m_values[(C * rows) + R];
    }

    template <int R, int C> static inline scalar_type read_element(Yukari::Maths::Matrix3 const &m)
    {
      return m.m_values[(C * rows) + R];
    }

    static inline scalar_type &write_element_idx(int r, int c, Yukari::Maths::Matrix3 &m)
    {
      return m.m_values[(c * rows) + r];
    }

    static inline scalar_type read_element_idx(int r, int c, Yukari::Maths::Matrix3 const &m)
    {
      return m.m_values[(c * rows) + r];
    }
  };
}
}
