/** @file */

#include "NDTIncrementalAlignment.h"

#include <pcl/common/transforms.h>

#include <YukariProcessing/SpatialOperations.h>

using namespace Yukari::Common;
using namespace Yukari::Processing;

namespace Yukari
{
namespace CaptureApp
{
  NDTIncrementalAlignment::NDTIncrementalAlignment()
      : m_logger(LoggingService::Instance().getLogger("NDTIncrementalAlignment"))
      , m_worldCloud()
  {
  }

  int NDTIncrementalAlignment::process(size_t frameNumber, CloudConstPtr cloud,
                                       IMU::IMUFrame_const_sptr imuFrame)
  {
    if (!(cloud && imuFrame))
    {
      m_logger->error("Do not have both cloud and IMU frame");
      return 1;
    }

    CloudPtr mutableCloud(new Cloud());

    /* Transform cloud */
    m_logger->trace("Transforming cloud by IMU");
    pcl::transformPointCloud(
        *cloud, *mutableCloud, imuFrame->position().toEigen(),
        SpatialOperations::RotateQuaternionForCloud(imuFrame->orientation().toEigen()));

    if (!m_worldCloud)
    {
      /* If this is the first recored cloud simply set it as he "world" cloud */
      m_worldCloud = CloudPtr(new Cloud(*mutableCloud));
    }
    else
    {
      /* Otherwise alignment is required */
      // TODO
    }

    return 0;
  }
}
}
