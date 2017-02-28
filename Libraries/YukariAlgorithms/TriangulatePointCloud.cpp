/** @file */

#include "TriangulatePointCloud.h"

#include <YukariCloudCapture/ICloudGrabber.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  TriangulatePointCloud::TriangulatePointCloud()
  {
    /* Add default validator */
    m_validator = [](const IAlgorithm &alg) {
      auto clouds = alg.getProperty(Processing::INPUT, "cloud");

      if (clouds.size() == 0)
        return "Must provide at least one point cloud";

      return "";
    };
  }

  void TriangulatePointCloud::execute()
  {
    Property cloud = getProperty(Processing::INPUT, "cloud");

    size_t len = cloud.size();

    Property mesh(len);

    for (size_t i = 0; i < len; i++)
    {
      ICloudGrabber::Cloud::Ptr c = cloud.value<ICloudGrabber::Cloud::Ptr>(i);

      // TODO
      // mesh[i] = ;
    }

    setProperty(Processing::OUTPUT, "mesh", mesh);
  }
}
}
