/** @file */

#include "ApplyTransformationToCloud.h"

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  ApplyTransformationToCloud::ApplyTransformationToCloud()
  {
    /* Add default validator */
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto clouds = inProps.find("clouds");
      if (clouds == inProps.end())
        return "No point clouds provided";

      auto transformations = inProps.find("transformations");
      if (transformations == inProps.end())
        return "No transformations provided";

      if (clouds->second.size() != transformations->second.size())
        return "Number of clouds and transformations must match";

      return "";
    };
  }

  void ApplyTransformationToCloud::execute()
  {
    /* TODO */
  }
}
}
