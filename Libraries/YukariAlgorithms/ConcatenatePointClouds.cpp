/** @file */

#include "ConcatenatePointClouds.h"

#include <YukariCloudCapture/ICloudGrabber.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  ConcatenatePointClouds::ConcatenatePointClouds()
  {
    /* Add default validator */
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto clouds = inProps.find("clouds");
      if (clouds == inProps.end())
        return "Input property \"clouds\" not found";

      if (clouds->second.size() == 0)
        return "Must provide at least one point cloud";

      return "";
    };
  }

  void ConcatenatePointClouds::execute()
  {
    Property cloud = getProperty(Processing::INPUT, "clouds");
    size_t len = cloud.size();

    ICloudGrabber::Cloud::Ptr outputCloud(
        new ICloudGrabber::Cloud(*cloud.value<ICloudGrabber::Cloud::Ptr>(0)));

    for (size_t i = 1; i < len; i++)
      *outputCloud += *cloud.value<ICloudGrabber::Cloud::Ptr>(i);

    setProperty(Processing::OUTPUT, "cloud", Property({outputCloud}));
  }
}
}
