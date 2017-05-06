/** @file */

#include "TaskPairWorldAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h>

#include "CloudOperations.h"
#include "PairRegistrationPointRepresentation.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskPairWorldAlignment::TaskPairWorldAlignment(const boost::filesystem::path &path,
                                       std::map<std::string, std::string> &params)
      : ITaskWorldAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskPairWorldAlignment"))
  {
  }

  void TaskPairWorldAlignment::doAlignment(Task t)
  {
    CloudPtr inputCloud(new Cloud());

    /* Transform cloud */
    m_logger->trace("Transforming cloud by IMU");
    pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->toCloudTransform());

    /* Downsample the input and world cloud for alignment */
    auto filteredInputCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
        inputCloud, m_voxelDownsamplePercentage);
    auto filteredWorldCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
        m_worldCloud, m_voxelDownsamplePercentage);

    /* Compute normals and curvature */
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals(new pcl::PointCloud<pcl::PointNormal>());

    pcl::NormalEstimation<PointT, pcl::PointNormal> normalEst;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    normalEst.setSearchMethod(tree);
    normalEst.setRadiusSearch(0.1);

    normalEst.setInputCloud(filteredInputCloud);
    normalEst.compute(*sourceNormals);

    normalEst.setInputCloud(filteredWorldCloud);
    normalEst.compute(*targetNormals);

    /* Init registration */
    PairRegistrationPointRepresentation pr;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    pr.setRescaleValues(alpha);

    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setMaximumIterations(50);
    reg.setTransformationEpsilon(1e-6);
    reg.setMaxCorrespondenceDistance(0.001);

    reg.setPointRepresentation(boost::make_shared<const PairRegistrationPointRepresentation>(pr));

    reg.setInputSource(sourceNormals);
    reg.setInputTarget(targetNormals);

    /* Align */
    pcl::PointCloud<pcl::PointNormal>::Ptr regResult(new pcl::PointCloud<pcl::PointNormal>());
    reg.align(*regResult);

    if (reg.hasConverged())
      m_logger->debug("Convergence reached");
    else
      m_logger->warn("Convergence not reached");
    m_logger->debug("Fitness score: {}", reg.getFitnessScore());

    Eigen::Matrix4f transform = reg.getFinalTransformation();
    m_logger->debug("Final transform: {}", transform);

    /* Translate full input cloud */
    pcl::transformPointCloud(*inputCloud, *inputCloud, transform);

    /* Add translated cloud to world cloud */
    *m_worldCloud += *inputCloud;
  }
}
}
