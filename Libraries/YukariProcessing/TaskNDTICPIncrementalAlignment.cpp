/** @file */

#include "TaskNDTICPIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskNDTICPIncrementalAlignment::TaskNDTICPIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskIncrementalAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskNDTICPIncrementalAlignment"))
  {
  }

  void TaskNDTICPIncrementalAlignment::doAlignment(Task t)
  {
    /* Downsample the input cloud for alignment */
    CloudPtr filteredInputCloud(new Cloud(*t.cloud));
    removeOutliers(filteredInputCloud, filteredInputCloud);
    downsample(filteredInputCloud, filteredInputCloud);

    /* Perform alignment */
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    setNDTParameters(ndt);

    ndt.setInputSource(filteredInputCloud);
    ndt.setInputTarget(m_previousCloud);

    /* Run alignment */
    CloudPtr transformedInputCloud(new Cloud());
    Eigen::Matrix4f initialGuess = t.imuFrame->toCloudTransform();
    ndt.align(*transformedInputCloud, initialGuess);

    if (ndt.hasConverged())
      m_logger->debug("NDT convergence reached");
    else
      m_logger->warn("NDT convergence not reached");
    m_logger->debug("Normal Distributions Transform score: {}", ndt.getFitnessScore());

    /* Perform ICP alignment */
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    setICPParameters(icp);

    icp.setInputSource(filteredInputCloud);
    icp.setInputTarget(m_previousCloud);

    /* Run alignment */
    initialGuess = t.imuFrame->toCloudTransform() * ndt.getFinalTransformation();
    icp.align(*transformedInputCloud, initialGuess);

    if (icp.hasConverged())
      m_logger->debug("ICP Convergence reached");
    else
      m_logger->warn("ICP convergence not reached");
    m_logger->debug("Iterative Closest Point score: {}", icp.getFitnessScore());

    /* Get transform from world origin to inoput cloud position */
    m_previousCloudWorldTransform = icp.getFinalTransformation() * m_previousCloudWorldTransform;
  }
}
}
