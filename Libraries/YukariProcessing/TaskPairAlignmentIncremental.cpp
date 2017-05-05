/** @file */

#include "TaskPairAlignmentIncremental.h"

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include "CloudOperations.h"
#include "PairRegistrationPointRepresentation.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskPairAlignmentIncremental::TaskPairAlignmentIncremental(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskPairAlignmentIncremental"))
      , m_saveTransforms(false)
      , m_saveClouds(false)
      , m_previousCloud()
      , m_previousCloudWorldTransform(Eigen::Matrix4f::Identity())
  {
    /* Parse save transforms option */
    std::string saveTransformParam =
        MapHelpers::Get<std::string, std::string>(params, "transform", "true");
    StringParsers::CleanString(saveTransformParam);
    m_saveTransforms = saveTransformParam == "true";

    /* Parse save clouds option */
    std::string saveCloudParam =
        MapHelpers::Get<std::string, std::string>(params, "cloud", "false");
    StringParsers::CleanString(saveCloudParam);
    m_saveClouds = saveCloudParam == "true";
  }

  int TaskPairAlignmentIncremental::process(Task t)
  {
    if (!(t.cloud && t.imuFrame))
    {
      m_logger->error("Do not have both cloud and IMU frame");
      return 1;
    }
    /* Format frame number */
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << t.frameNumber;
    std::string frameNoStr = ss.str();

    if (!m_previousCloud)
    {
      /* Set initial transform */
      m_previousCloudWorldTransform = t.imuFrame->toCloudTransform();
    }
    else
    {
      /* Downsample the input and world cloud for alignment */
      auto filteredInputCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
          t.cloud, m_voxelDownsamplePercentage);
      auto filteredWorldCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
          m_previousCloud, m_voxelDownsamplePercentage);

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
      float alpha[4] = {1.0, 1.0, 1.0, 1.0};
      pr.setRescaleValues(alpha);

      pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
      reg.setMaximumIterations(50);
      reg.setTransformationEpsilon(1e-9);
      reg.setMaxCorrespondenceDistance(0.0005);

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

      m_previousCloudWorldTransform = reg.getFinalTransformation();
      m_logger->debug("Final transform: {}", m_previousCloudWorldTransform);
    }

    /* Set previous cloud */
    m_previousCloud = CloudPtr(new Cloud(*t.cloud));

    /* Transform the previous/target cloud by it's world position (for next frame) */
    pcl::transformPointCloud<PointT>(*m_previousCloud, *m_previousCloud,
                                     m_previousCloudWorldTransform);

    /* Save transformed cloud */
    if (m_saveTransforms)
    {
      boost::filesystem::path cloudFilename = m_outputDirectory / (frameNoStr + "_cloud.pcd");
      m_logger->trace("Saving transformed point cloud for frame {}: {}", t.frameNumber,
                      cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_previousCloud);
    }

    /* Save transformation */
    if (m_saveClouds)
    {
      IMU::IMUFrame transformFrame(m_previousCloudWorldTransform);
      boost::filesystem::path imuFilename = m_outputDirectory / (frameNoStr + "_transform.txt");
      transformFrame.save(imuFilename);
    }

    return 0;
  }
}
}