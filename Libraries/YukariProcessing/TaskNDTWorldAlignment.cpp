/** @file */

#include "TaskNDTWorldAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include "CloudOperations.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskNDTWorldAlignment::TaskNDTWorldAlignment(const boost::filesystem::path &path,
                                               std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskNDTWorldAlignment"))
      , m_worldCloud()
  {
  }

  int TaskNDTWorldAlignment::process(Task t)
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
      /* If this is the first recored cloud simply set it as he "world" cloud */
      m_worldCloud = CloudPtr(new Cloud(*inputCloud));
    }
    else
    {
      /* Otherwise alignment is required */

      /* Downsample the input cloud for alignment */
      auto filteredInputCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
          inputCloud, m_voxelDownsamplePercentage);

      /* Perform alignment */
      pcl::NormalDistributionsTransform<PointT, PointT> ndt;
      setNDTParameters(ndt);

      ndt.setInputSource(filteredInputCloud);
      ndt.setInputTarget(m_worldCloud);

      /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
      CloudPtr transformedInputCloud(new Cloud());
      ndt.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

      if (ndt.hasConverged())
        m_logger->debug("Convergence reached");
      else
        m_logger->warn("Convergence not reached");
      m_logger->debug("After {} iterations", ndt.getFinalNumIteration());
      m_logger->debug("Normal Distributions Transform score: {}", ndt.getFitnessScore());

      /* Translate full input cloud */
      pcl::transformPointCloud(*inputCloud, *transformedInputCloud, ndt.getFinalTransformation());

      /* Add translated cloud to world cloud */
      *m_worldCloud += *transformedInputCloud;
    }

    return 0;
  }

  int TaskNDTWorldAlignment::onStop()
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