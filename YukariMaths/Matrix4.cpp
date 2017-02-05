/** @file */

#include "Matrix4.h"

#include <boost/qvm/all.hpp>

using namespace boost::qvm;

namespace Yukari
{
namespace Maths
{
  Matrix4::Matrix4()
  {
    boost::qvm::set_identity(*this);
  }

  std::ostream &operator<<(std::ostream &s, const Matrix4 &m)
  {
    const float *v = m.m_values;

    // clang-format off
    s << '[' << v[0] << ", " << v[4] << ", " << v[8] << ", " << v[12] << ",\n"
      << ' ' << v[1] << ", " << v[5] << ", " << v[9] << ", " << v[13] << ",\n"
      << ' ' << v[2] << ", " << v[6] << ", " << v[10] << ", " << v[14] << ",\n"
      << ' ' << v[3] << ", " << v[7] << ", " << v[11] << ", " << v[15] << ']';
    // clang-format on

    return s;
  }
}
}
