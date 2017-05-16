/** @file */

#include "TaskICPIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskICPIncrementalAlignment::TaskICPIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskIncrementalAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskICPIncrementalAlignment"))
  {
  }

  void TaskICPIncrementalAlignment::doAlignment(Task t)
  {
    /* Downsample the input cloud for alignment */
	CloudPtr filteredInputCloud(new Cloud);
	downsample(t.cloud, filteredInputCloud);

    /* Perform alignment */
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    setICPParameters(icp);

    icp.setInputSource(filteredInputCloud);
    icp.setInputTarget(m_previousCloud);

    /* Run alignment */
    CloudPtr transformedInputCloud(new Cloud());
    Eigen::Matrix4f initialGuess = t.imuFrame->toCloudTransform();
    icp.align(*transformedInputCloud, initialGuess);

    if (icp.hasConverged())
      m_logger->debug("Convergence reached");
    else
      m_logger->warn("Convergence not reached");
    m_logger->debug("Fitness score: {}", icp.getFitnessScore());

    /* Get transform from world origin to input cloud position */
    m_previousCloudWorldTransform = icp.getFinalTransformation();
  }
}
}
