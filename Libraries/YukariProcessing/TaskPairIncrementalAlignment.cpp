/** @file */

#include "TaskPairIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h>

#include "PairRegistrationPointRepresentation.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskPairIncrementalAlignment::TaskPairIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskIncrementalAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskPairIncrementalAlignment"))
  {
  }

  void TaskPairIncrementalAlignment::doAlignment(Task t)
  {
    /* Downsample the input and world cloud for alignment */
    CloudPtr filteredInputCloud(new Cloud(*t.cloud));
    removeOutliers(filteredInputCloud, filteredInputCloud);
    downsample(filteredInputCloud, filteredInputCloud);
    CloudPtr filteredWorldCloud(new Cloud);
    downsample(m_previousCloud, filteredWorldCloud);

    /* Compute normals and curvature */
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals(new pcl::PointCloud<pcl::PointNormal>());

    pcl::NormalEstimation<PointT, pcl::PointNormal> normalEst;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    normalEst.setSearchMethod(tree);
    // normalEst.setRadiusSearch(0.1);
    normalEst.setKSearch(30);

    normalEst.setInputCloud(filteredInputCloud);
    normalEst.compute(*sourceNormals);

    normalEst.setInputCloud(filteredWorldCloud);
    normalEst.compute(*targetNormals);

    /* Init registration */
    PairRegistrationPointRepresentation pr;
    float alpha[4] = {1.0f, 1.0f, 1.0f, 1.0f};
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
    Eigen::Matrix4f initialGuess = t.imuFrame->toCloudTransform();
    reg.align(*regResult, initialGuess);

    if (reg.hasConverged())
      m_logger->debug("Convergence reached");
    else
      m_logger->warn("Convergence not reached");
    m_logger->debug("Fitness score: {}", reg.getFitnessScore());

    m_previousCloudWorldTransform = reg.getFinalTransformation() * m_previousCloudWorldTransform;
    m_logger->debug("Final transform: {}", m_previousCloudWorldTransform);
  }
}
}
