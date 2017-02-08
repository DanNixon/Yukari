/** @file */

#pragma once

#include "BaseMathType.h"

#include <boost/qvm/vec_traits.hpp>

namespace Yukari
{
namespace Maths
{
  class Vector4 : public BaseMathType<4>
  {
  public:
    Vector4(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 1.0f);

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

    inline float w() const
    {
      return m_values[3];
    }

    inline float &w()
    {
      return m_values[3];
    }

    friend struct boost::qvm::vec_traits<Yukari::Maths::Vector4>;
  };
}
}

namespace boost
{
namespace qvm
{
  template <> struct vec_traits<Yukari::Maths::Vector4>
  {
    static int const dim = 4;
    typedef float scalar_type;

    template <int I> static inline scalar_type &write_element(Yukari::Maths::Vector4 &v)
    {
      return v.m_values[I];
    }

    template <int I> static inline scalar_type read_element(Yukari::Maths::Vector4 const &v)
    {
      return v.m_values[I];
    }

    static inline scalar_type &write_element_idx(int i, Yukari::Maths::Vector4 &v)
    {
      return v.m_values[i];
    }

    static inline scalar_type read_element_idx(int i, Yukari::Maths::Vector4 const &v)
    {
      return v.m_values[i];
    }
  };
}
}
