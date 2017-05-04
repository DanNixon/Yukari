/** @file */

#include "TaskAppendTransformedClouds.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskAppendTransformedClouds::TaskAppendTransformedClouds(const boost::filesystem::path &path)
      : IFrameProcessingTask(path)
      , m_logger(LoggingService::Instance().getLogger("TaskAppendTransformedClouds"))
      , m_worldCloud()
  {
  }

  int TaskAppendTransformedClouds::process(Task t)
  {
    if (!(t.cloud && t.imuFrame))
    {
      m_logger->error("Do not have both cloud and IMU frame");
      return 1;
    }

    CloudPtr inputCloud(new Cloud());

    /* Transform cloud */
    m_logger->trace("Transforming cloud by IMU");
    pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->toCloudTransform());

    if (!m_worldCloud)
    {
      /* If this is the first recored cloud simply set it as the "world" cloud */
      m_worldCloud = CloudPtr(new Cloud(*inputCloud));
    }
    else
    {
      /* Add translated cloud to world cloud */
      *m_worldCloud += *inputCloud;
    }

    return 0;
  }

  int TaskAppendTransformedClouds::onStop()
  {
    if (!m_worldCloud)
    {
      m_logger->warn("No world cloud, nothing saved");
      return 1;
    }

    /* Generate filename */
    boost::filesystem::path cloudFilename = m_outputDirectory / "world_cloud.pcd";

    /* Save world cloud */
    m_logger->trace("Saving world point cloud: {}", cloudFilename);
    pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_worldCloud);

    return 0;
  }
}
}