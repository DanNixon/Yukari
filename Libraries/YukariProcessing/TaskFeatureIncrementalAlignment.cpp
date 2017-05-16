/** @file */

#include "TaskFeatureIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskFeatureIncrementalAlignment::TaskFeatureIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskIncrementalAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskFeatureIncrementalAlignment"))
      , m_debugSave(true)
  {
  }

  int TaskFeatureIncrementalAlignment::process(Task t)
  {
    if (!t.cloud)
    {
      m_logger->error("Do not have point cloud");
      return 1;
    }

    if (!m_previousCloud)
    {
      /* Set initial transform */
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
      boost::filesystem::path cloudFilename = formatFilename(t, "_cloud.pcd");
      m_logger->trace("Saving transformed point cloud for frame {}: {}", t.frameNumber,
                      cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_previousCloud);
    }

    /* Save transformation */
    if (m_saveTransforms)
    {
      IMU::IMUFrame transformFrame(m_previousCloudWorldTransform);
      transformFrame.save(formatFilename(t, "_transform.txt"));
    }

    return 0;
  }

  TaskFeatureIncrementalAlignment::SingleCloudData
  TaskFeatureIncrementalAlignment::preProcessSingleCloud(Task t)
  {
    m_logger->debug("{} points in input cloud", t.cloud->size());

    SingleCloudData d;
    d.downsampled = boost::make_shared<Cloud>();
    d.normals = boost::make_shared<NormalCloud>();
    d.features = boost::make_shared<FeatureCloud>();

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    CloudPtr inCloud;

    /* Filter outliers */
    inCloud = boost::make_shared<Cloud>(*t.cloud);
    removeOutliers(t.cloud, inCloud);

    /* Downsample cloud */
    downsample(inCloud, d.downsampled);

    /* Normal estimation */
    m_logger->trace("Normal estimation");
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(m_normalEstimationRadiusSearch);
    ne.setInputCloud(d.downsampled);
    ne.compute(*d.normals);

    for (int i = 0; i < d.normals->points.size(); i++)
    {
      if (!pcl::isFinite<pcl::Normal>(d.normals->points[i]))
        m_logger->warn("normals[{}] is not finite", i);
    }

    /* Save point cloud with normals for debugging */
    if (m_debugSave)
    {
      auto filename = formatFilename(t, "_downsampledWithNormals.pcd");
      m_logger->debug("Saving downsampled cloud with normals: {}", filename);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(
          new pcl::PointCloud<pcl::PointNormal>);
      pcl::copyPointCloud(*d.downsampled, *cloud);
      pcl::concatenateFields(*cloud, *d.normals, *cloudWithNormals);
      pcl::io::savePCDFileBinaryCompressed(filename.string(), *cloudWithNormals);
    }

    /* Feature estimation */
    m_logger->trace("Feature estimation");
    pcl::FPFHEstimationOMP<PointT, NormalT, FeatureT> pfh;
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(m_featureRadiusSearch);
    pfh.setInputCloud(d.downsampled);
    pfh.setInputNormals(d.normals);
    pfh.compute(*d.features);
    m_logger->debug("{} points in feature cloud", d.features->size());

    /* Save feature cloud with normals for debugging */
    if (m_debugSave)
    {
      auto filename = formatFilename(t, "_features.pcd");
      m_logger->debug("Saving feature cloud: {}", filename);
      pcl::io::savePCDFileBinaryCompressed(filename.string(), *d.features);
    }

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
    sac.setMaxIterations(m_corrRejectMaxIters);
    sac.setInlierThreshold(m_correRejectInlierThreshold);
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

    /* Save trimmed clouds for debugging */
    if (m_debugSave)
    {
      {
        auto filename = formatFilename(t, "_trimmedInput.pcd");
        m_logger->debug("Saving trimmed input cloud: {}", filename);
        pcl::io::savePCDFileBinaryCompressed(filename.string(), *trimmedInput);
      }
      {
        auto filename = formatFilename(t, "_trimmedTarget.pcd");
        m_logger->debug("Saving trimmed target cloud: {}", filename);
        pcl::io::savePCDFileBinaryCompressed(filename.string(), *trimmedTarget);
      }
    }

    /* Fine alignment (ICP) */
    m_logger->trace("Fine alignment");
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    setICPParameters(icp);
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
