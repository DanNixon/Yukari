/** @file */

#pragma once

namespace Yukari
{
namespace Maths
{
  class BaseMathType
  {
  public:
    BaseMathType(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 0.0f)
        : m_x(x)
        , m_y(y)
        , m_z(z)
        , m_w(w)
    {
    }

    BaseMathType(const BaseMathType &o)
        : m_x(o.m_x)
        , m_y(o.m_y)
        , m_z(o.m_z)
        , m_w(o.m_w)
    {
    }

    virtual ~BaseMathType()
    {
    }

    inline bool operator==(const BaseMathType &other) const
    {
      return (m_x == other.m_x) && (m_y == other.m_y) && (m_z == other.m_z) &&
             (m_w == other.m_w);
    }

    inline bool operator!=(const BaseMathType &other) const
    {
      return !this->operator==(other);
    }

    void toZero()
    {
      m_x = 0.0f;
      m_y = 0.0f;
      m_z = 0.0f;
      m_w = 0.0f;
    }

    float w() const
    {
      return m_w;
    }

  protected:
    float m_x;
    float m_y;
    float m_z;
    float m_w;
  };
}
}
