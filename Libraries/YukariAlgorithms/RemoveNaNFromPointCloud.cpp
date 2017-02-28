/** @file */

#include "RemoveNaNFromPointCloud.h"

#include <YukariCloudCapture/ICloudGrabber.h>
#include <pcl/filters/filter.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  RemoveNaNFromPointCloud::RemoveNaNFromPointCloud()
  {
    /* Add default validator */
    m_validator = [](const IAlgorithm &alg) {
      Property_sptr clouds = alg.getProperty(Processing::INPUT, "cloud");

      if (clouds->size() == 0)
        return "Must provide at least one point cloud";

      return "";
    };
  }

  void RemoveNaNFromPointCloud::doExecute()
  {
    Property_sptr cloud = getProperty(Processing::INPUT, "cloud");
    size_t len = cloud->size();
    Property_sptr filteredCloud = std::make_shared<Property>(len);

    for (size_t i = 0; i < len; i++)
    {
      ICloudGrabber::Cloud::Ptr c = cloud->value<ICloudGrabber::Cloud::Ptr>(i);
      ICloudGrabber::Cloud::Ptr fc(new ICloudGrabber::Cloud());

      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*c, *fc, indices);

      (*filteredCloud)[i] = fc;
    }

    setProperty(Processing::OUTPUT, "cloud", filteredCloud);
  }
}
}
