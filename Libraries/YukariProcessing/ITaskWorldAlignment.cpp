/** @file */

#include "ITaskWorldAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  ITaskWorldAlignment::ITaskWorldAlignment(const boost::filesystem::path &path,
                                           std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("ITaskWorldAlignment"))
      , m_worldCloud()
  {
  }

  int ITaskWorldAlignment::process(Task t)
  {
    if (!(t.cloud && t.imuFrame))
    {
      m_logger->error("Do not have both cloud and IMU frame");
      return 1;
    }

    if (!m_worldCloud)
    {
      CloudPtr inputCloud(new Cloud());

      /* Transform cloud */
      m_logger->trace("Transforming cloud by IMU");
      pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->toCloudTransform());

      /* If this is the first recored cloud simply set it as he "world" cloud */
      m_worldCloud = inputCloud;
    }
    else
    {
      doAlignment(t);
    }

    return 0;
  }

  int ITaskWorldAlignment::onStop()
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
