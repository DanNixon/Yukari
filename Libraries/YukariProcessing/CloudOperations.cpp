/** @file */

#include "CloudOperations.h"

#include <Eigen/Geometry>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

using namespace Yukari::CloudCapture;

namespace Yukari
{
namespace Processing
{
  ICloudGrabber::Cloud::Ptr
  CloudOperations::ApplyTransformationToCloud(ICloudGrabber::Cloud::Ptr cloud,
                                              Eigen::Matrix4f transform)
  {
    ICloudGrabber::Cloud::Ptr tc(new ICloudGrabber::Cloud());
    pcl::transformPointCloud(*cloud, *tc, transform);
    return tc;
  }

  ICloudGrabber::Cloud::Ptr
  CloudOperations::ConcatenateClouds(std::vector<ICloudGrabber::Cloud::Ptr> clouds)
  {
    if (clouds.empty())
      return nullptr;

    if (clouds.size() == 1)
      return clouds.front();

    ICloudGrabber::Cloud::Ptr outputCloud(new ICloudGrabber::Cloud(*(clouds.front())));

    for (auto it = clouds.begin() + 1; it != clouds.end(); ++it)
      *outputCloud += *(*it);

    return outputCloud;
  }

  ICloudGrabber::Cloud::Ptr CloudOperations::RemoveNaNFromCloud(ICloudGrabber::Cloud::Ptr cloud)
  {
    ICloudGrabber::Cloud::Ptr fc(new ICloudGrabber::Cloud());

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *fc, indices);

    return fc;
  }
}
}
