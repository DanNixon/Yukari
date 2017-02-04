/** @file */

#include "Vector3.h"

namespace Yukari
{
namespace Maths
{
  Vector3::Vector3(float x, float y, float z)
      : BaseVectorType()
  {
    m_values[0] = x;
    m_values[1] = y;
    m_values[2] = z;
  }

}
}
