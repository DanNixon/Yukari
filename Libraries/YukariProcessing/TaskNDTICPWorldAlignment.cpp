/** @file */

#include "TaskNDTICPWorldAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskNDTICPWorldAlignment::TaskNDTICPWorldAlignment(const boost::filesystem::path &path,
                                                     std::map<std::string, std::string> &params)
      : ITaskWorldAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskNDTICPWorldAlignment"))
  {
  }

  void TaskNDTICPWorldAlignment::doAlignment(Task t)
  {
    CloudPtr inputCloud(new Cloud());

    /* Transform cloud */
    m_logger->trace("Transforming cloud by IMU");
    pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->toCloudTransform());

    /* Downsample the input cloud for alignment */
	CloudPtr filteredInputCloud(new Cloud);
	downsample(inputCloud, filteredInputCloud);

    /* Perform NDT alignment */
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    setNDTParameters(ndt);

    ndt.setInputSource(filteredInputCloud);
    ndt.setInputTarget(m_worldCloud);

    /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
    CloudPtr transformedInputCloud(new Cloud());
    ndt.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

    if (ndt.hasConverged())
      m_logger->debug("NDT convergence reached");
    else
      m_logger->warn("NDT convergence not reached");
    m_logger->debug("After {} NCP iterations", ndt.getFinalNumIteration());
    m_logger->debug("Normal Distributions Transform score: {}", ndt.getFitnessScore());

    /* Perform ICP alignment */
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    setICPParameters(icp);

    icp.setInputSource(filteredInputCloud);
    icp.setInputTarget(m_worldCloud);

    /* Run alignment (take nitial guess from NDT) */
    icp.align(*transformedInputCloud, ndt.getFinalTransformation());

    if (icp.hasConverged())
      m_logger->debug("ICP convergence reached");
    else
      m_logger->warn("ICP convergence not reached");
    m_logger->debug("Iterative Closest Point score: {}", icp.getFitnessScore());

    /* Translate full input cloud */
    pcl::transformPointCloud(*inputCloud, *transformedInputCloud, icp.getFinalTransformation());

    /* Add translated cloud to world cloud */
    *m_worldCloud += *transformedInputCloud;
  }
}
}
