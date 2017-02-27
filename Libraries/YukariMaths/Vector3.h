/** @file */

#pragma once

#include "BaseMathType.h"

#include <Eigen/Geometry>
#include <boost/qvm/vec_traits.hpp>

namespace Yukari
{
namespace Maths
{
  class Vector3 : public BaseMathType<3>
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

    inline Eigen::Vector3f toEigen() const
    {
      return Eigen::Vector3f(m_values[0], m_values[1], m_values[2]);
    }

    friend struct boost::qvm::vec_traits<Yukari::Maths::Vector3>;
  };
}
}

namespace boost
{
namespace qvm
{
  template <> struct vec_traits<Yukari::Maths::Vector3>
  {
    static int const dim = 3;
    typedef float scalar_type;

    template <int I> static inline scalar_type &write_element(Yukari::Maths::Vector3 &v)
    {
      return v.m_values[I];
    }

    template <int I> static inline scalar_type read_element(Yukari::Maths::Vector3 const &v)
    {
      return v.m_values[I];
    }

    static inline scalar_type &write_element_idx(int i, Yukari::Maths::Vector3 &v)
    {
      return v.m_values[i];
    }

    static inline scalar_type read_element_idx(int i, Yukari::Maths::Vector3 const &v)
    {
      return v.m_values[i];
    }
  };
}
}
