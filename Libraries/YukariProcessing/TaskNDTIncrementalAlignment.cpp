/** @file */

#include "TaskNDTIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskNDTIncrementalAlignment::TaskNDTIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskIncrementalAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskNDTIncrementalAlignment"))
  {
  }

  void TaskNDTIncrementalAlignment::doAlignment(Task t)
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
      m_logger->debug("Convergence reached");
    else
      m_logger->warn("Convergence not reached");
    m_logger->debug("Normal Distributions Transform score: {}", ndt.getFitnessScore());

    /* Get transform from world origin to inoput cloud position */
    m_previousCloudWorldTransform = ndt.getFinalTransformation() * m_previousCloudWorldTransform;
  }
}
}
