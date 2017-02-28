/** @file */

#include "LoadPointCloud.h"

#include <Eigen/Geometry>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <pcl/common/transforms.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  LoadPointCloud::LoadPointCloud()
  {
    /* Add default validator */
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto file = inProps.find("file");
      if (file == inProps.end())
        return "No file provided";

      return "";
    };
  }

  void LoadPointCloud::execute()
  {
    Property file = getProperty(Processing::INPUT, "file");

    size_t len = file.size();

    Property cloud(len);

    for (size_t i = 0; i < len; i++)
    {
      const std::string filename = file.value<std::string>(i);

      /* TODO */
      /* cloud[i] = tc; */
    }

    setProperty(Processing::OUTPUT, "cloud", cloud);
  }
}
}
