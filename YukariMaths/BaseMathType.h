/** @file */

#pragma once

#include <iostream>
#include <stdexcept>

namespace Yukari
{
namespace Maths
{
  template <size_t SIZE> class BaseMathType
  {
  public:
    BaseMathType()
    {
      toZero();
    }

    BaseMathType(const BaseMathType &o)
    {
      for (size_t i = 0; i < SIZE; i++)
        m_values[i] = o.m_values[i];
    }

    virtual ~BaseMathType()
    {
    }

    inline bool operator==(const BaseMathType &other) const
    {
      for (size_t i = 0; i < SIZE; i++)
      {
        if (m_values[i] != other.m_values[i])
          return false;
      }

      return true;
    }

    inline bool operator!=(const BaseMathType &other) const
    {
      return !this->operator==(other);
    }

    inline void toZero()
    {
      for (size_t i = 0; i < SIZE; i++)
        m_values[i] = 0.0f;
    }

    inline float operator[](size_t idx) const
    {
      if (idx > SIZE)
        throw std::runtime_error("Math type index out of range.");

      return m_values[idx];
    }

    inline float &operator[](size_t idx)
    {
      if (idx > SIZE)
        throw std::runtime_error("Math type index out of range.");

      return m_values[idx];
    }

    friend std::ostream &operator<<(std::ostream &s, const BaseMathType &o)
    {
      s << '[';

      for (size_t i = 0; i < SIZE; i++)
      {
        if (i > 0)
          s << ", ";
        s << o.m_values[i];
      }

      s << ']';

      return s;
    }

    friend std::istream &operator>>(std::istream &s, BaseMathType &o)
    {
      const int n = 50;

      o = BaseMathType();

      s.ignore(n, '[');
      s >> o.m_values[0];
      s.ignore(n, ',');
      s >> o.m_values[1];
      s.ignore(n, ',');
      s >> o.m_values[2];
      s.ignore(n, ',');
      s >> o.m_values[3];
      s.ignore(n, ']');

      return s;
    }

  protected:
    float m_values[SIZE];
  };
}
}
