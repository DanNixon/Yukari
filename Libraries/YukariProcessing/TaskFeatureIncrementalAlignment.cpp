/** @file */

#include "TaskFeatureIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include "CloudOperations.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskFeatureIncrementalAlignment::TaskFeatureIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskIncrementalAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskICPIncrementalAlignment"))
  {
    /* Parse IMU estimate option */
    {
      std::string paramStr = MapHelpers::Get<std::string, std::string>(params, "imu", "false");
      StringParsers::CleanString(paramStr);
      m_useIMUEstimate = paramStr == "true";
    }
  }

  int TaskFeatureIncrementalAlignment::process(Task t)
  {
    if (!t.cloud)
    {
      m_logger->error("Do not have point cloud");
      return 1;
    }

    if (m_useIMUEstimate && !t.imuFrame)
    {
      m_logger->error("Do not have IMU frame");
      return 1;
    }

    /* Format frame number */
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << t.frameNumber;
    std::string frameNoStr = ss.str();

    if (!m_previousCloud)
    {
      /* Set initial transform */
      if (m_useIMUEstimate)
        m_previousCloudWorldTransform = t.imuFrame->toCloudTransform();
      else
        m_previousCloudWorldTransform = Eigen::Matrix4f::Identity();

      m_targetData = preProcessSingleCloud(t);
    }
    else
    {
      doAlignment(t);
    }

    /* Set previous cloud */
    m_previousCloud = CloudPtr(new Cloud(*t.cloud));

    /* Transform the previous/target cloud by it's world position (for next frame) */
    pcl::transformPointCloud<PointT>(*m_previousCloud, *m_previousCloud,
                                     m_previousCloudWorldTransform);

    /* Save transformed cloud */
    if (m_saveClouds)
    {
      boost::filesystem::path cloudFilename = m_outputDirectory / (frameNoStr + "_cloud.pcd");
      m_logger->trace("Saving transformed point cloud for frame {}: {}", t.frameNumber,
                      cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_previousCloud);
    }

    /* Save transformation */
    if (m_saveTransforms)
    {
      IMU::IMUFrame transformFrame(m_previousCloudWorldTransform);
      boost::filesystem::path imuFilename = m_outputDirectory / (frameNoStr + "_transform.txt");
      transformFrame.save(imuFilename);
    }

    return 0;
  }

  TaskFeatureIncrementalAlignment::SingleCloudData
  TaskFeatureIncrementalAlignment::preProcessSingleCloud(Task t)
  {
    SingleCloudData d;
    d.downsampled = boost::make_shared<Cloud>();
    d.normals = boost::make_shared<NormalCloud>();
    d.features = boost::make_shared<FeatureCloud>();

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    /* Downsample cloud */
    m_logger->trace("Downsampling (leaf size: {})", m_voxelDownsamplePercentage);
    pcl::ApproximateVoxelGrid<PointT> voxelFilter;
    voxelFilter.setLeafSize(m_voxelDownsamplePercentage, m_voxelDownsamplePercentage,
                            m_voxelDownsamplePercentage);
    voxelFilter.setInputCloud(t.cloud);
    voxelFilter.filter(*d.downsampled);

    /* Normal estimation */
    m_logger->trace("Normal estimation");
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.setInputCloud(d.downsampled);
    ne.compute(*d.normals);

    for (int i = 0; i < d.normals->points.size(); i++)
    {
      if (!pcl::isFinite<pcl::Normal>(d.normals->points[i]))
        m_logger->warn("normals[{}] is not finite", i);
    }

    /* Feature estimation */
    m_logger->trace("Feature estimation");
    pcl::FPFHEstimationOMP<PointT, NormalT, FeatureT> pfh;
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(0.05);
    pfh.setInputCloud(d.downsampled);
    pfh.setInputNormals(d.normals);
    pfh.compute(*d.features);

    return d;
  }

  void TaskFeatureIncrementalAlignment::doAlignment(Task t)
  {
    SingleCloudData inputData = preProcessSingleCloud(t);

    /* Correspondance estimation */
    m_logger->trace("Correspondance estimation");
    pcl::registration::CorrespondenceEstimation<FeatureT, FeatureT> ce;
    ce.setInputTarget(m_targetData.features);
    ce.setInputCloud(inputData.features);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    ce.determineCorrespondences(*correspondences);
    m_logger->debug("Num correspondences = {}", correspondences->size());

    /* Correspondence rejection */
    m_logger->trace("Correspondence rejection");
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
    sac.setMaxIterations(1000);
    sac.setInlierThreshold(0.2);
    sac.setTargetCloud(m_targetData.downsampled);
    sac.setInputCloud(inputData.downsampled);
    sac.setInputCorrespondences(correspondences);
    pcl::CorrespondencesPtr keptCorrespondences(new pcl::Correspondences());
    sac.getCorrespondences(*keptCorrespondences);
    m_logger->debug("Num correspondences after rejection = {}", keptCorrespondences->size());

    /* Initial transformation estimation */
    m_logger->trace("Initial transformation estimation");
    Eigen::Matrix4f transform;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
    svd.estimateRigidTransformation(*inputData.downsampled, *m_targetData.downsampled,
                                    *keptCorrespondences, transform);

    /* Trim clouds to correspondences */
    m_logger->trace("Trim clouds to correspondences");
    CloudPtr trimmedInput(new Cloud);
    CloudPtr trimmedTarget(new Cloud);
    size_t idxC;
    size_t checkLim = std::min(m_targetData.downsampled->size(), inputData.downsampled->size());
    for (size_t i = 0; i < keptCorrespondences->size(); ++i)
    {
      idxC = keptCorrespondences->at(i).index_query;
      if (idxC < checkLim)
      {
        PointT &ptI = inputData.downsampled->at(idxC);
        PointT &ptT = m_targetData.downsampled->at(idxC);
        trimmedInput->push_back(ptI);
        trimmedTarget->push_back(ptT);
      }
      else
      {
        m_logger->warn("Point index error: {}", idxC);
      }
    }
    m_logger->debug("Points in trimmed input cloud = {}", trimmedInput->size());
    m_logger->debug("Points in trimmed target cloud = {}", trimmedTarget->size());

    /* Fine alignment (ICP) */
    m_logger->trace("Fine alignment\n");
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setInputTarget(trimmedTarget);
    icp.setInputCloud(trimmedInput);
    CloudPtr registeredInput(new Cloud);
    icp.align(*registeredInput, transform);
    m_logger->debug("ICP has converged: {}", icp.hasConverged());
    m_logger->debug("ICP fitness score = {}", icp.getFitnessScore());
    m_previousCloudWorldTransform = icp.getFinalTransformation();

    m_targetData = inputData;
  }
}
}
