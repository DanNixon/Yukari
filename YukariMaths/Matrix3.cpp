/** @file */

#include "Matrix3.h"

#include <boost/qvm/all.hpp>

using namespace boost::qvm;

namespace Yukari
{
namespace Maths
{
  Matrix3::Matrix3()
  {
    boost::qvm::set_identity(*this);
  }

  std::ostream &operator<<(std::ostream &s, const Matrix3 &m)
  {
    const float *v = m.m_values;

    // clang-format off
    s << '[' << v[0] << ", " << v[3] << ", " << v[6] << ",\n"
      << ' ' << v[1] << ", " << v[4] << ", " << v[7] << ",\n"
      << ' ' << v[2] << ", " << v[5] << ", " << v[8] << ']';
    // clang-format on

    return s;
  }
}
}
