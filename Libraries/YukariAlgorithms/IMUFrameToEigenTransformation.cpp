/** @file */

#include "IMUFrameToEigenTransformation.h"

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  IMUFrameToEigenTransformation::IMUFrameToEigenTransformation()
  {
    /* Add default validator */
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto frames = inProps.find("frames");
      if (frames == inProps.end())
        return "Frame property not provided";

      if (frames->second.size() == 0)
        return "Must provide at least one frame";

      return "";
    };
  }

  void IMUFrameToEigenTransformation::execute()
  {
    /* TODO */
  }
}
}
