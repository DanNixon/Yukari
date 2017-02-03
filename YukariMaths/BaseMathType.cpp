
/** @file */

#include "BaseMathType.h"

namespace Yukari
{
namespace Maths
{
  std::ostream &operator<<(std::ostream &s, const BaseMathType &o)
  {
    s << '[' << o.m_x << ", " << o.m_y << ", " << o.m_z << ", " << o.m_w << ']';
    return s;
  }

  std::istream &operator>>(std::istream &s, BaseMathType &o)
  {
    const int n = 50;

    float x, y, z, w;

    s.ignore(n, '[');
    s >> x;
    s.ignore(n, ',');
    s >> y;
    s.ignore(n, ',');
    s >> z;
    s.ignore(n, ',');
    s >> w;
    s.ignore(n, ']');

    o = BaseMathType(x, y, z, w);

    return s;
  }
}
}
