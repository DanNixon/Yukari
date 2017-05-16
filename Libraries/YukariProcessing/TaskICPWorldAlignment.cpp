/** @file */

#include "TaskICPWorldAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskICPWorldAlignment::TaskICPWorldAlignment(const boost::filesystem::path &path,
                                               std::map<std::string, std::string> &params)
      : ITaskWorldAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskICPWorldAlignment"))
  {
  }

  void TaskICPWorldAlignment::doAlignment(Task t)
  {
    CloudPtr inputCloud(new Cloud());

    /* Transform cloud */
    m_logger->trace("Transforming cloud by IMU");
    pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->toCloudTransform());

    /* Downsample the input cloud for alignment */
	CloudPtr filteredInputCloud(new Cloud);
	downsample(inputCloud, filteredInputCloud);

    /* Perform alignment */
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    setICPParameters(icp);

    icp.setInputSource(filteredInputCloud);
    icp.setInputTarget(m_worldCloud);

    /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
    CloudPtr transformedInputCloud(new Cloud());
    icp.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

    if (icp.hasConverged())
      m_logger->debug("Convergence reached");
    else
      m_logger->warn("Convergence not reached");
    m_logger->debug("Fitness score: {}", icp.getFitnessScore());

    /* Translate full input cloud */
    pcl::transformPointCloud(*inputCloud, *transformedInputCloud, icp.getFinalTransformation());

    /* Add translated cloud to world cloud */
    *m_worldCloud += *transformedInputCloud;
  }
}
}
