/** @file */

#include "ITaskAlignment.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <YukariCommon/LoggingService.h>
#include <YukariCommon/MapHelpers.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  ITaskAlignment::ITaskAlignment(const boost::filesystem::path &path,
                                 std::map<std::string, std::string> &params)
      : IFrameProcessingTask(path)
      , m_logger(LoggingService::Instance().getLogger("ITaskAlignment"))
      , m_outlierRemoval(
            std::stoi(MapHelpers::Get<std::string, std::string>(params, "orenable", "0")))
      , m_outlierRemovalMeanK(
            std::stoi(MapHelpers::Get<std::string, std::string>(params, "ormeank", "50")))
      , m_outlierRemovalStdDevMulThr(
            std::stof(MapHelpers::Get<std::string, std::string>(params, "orstddev", "1.0")))
      , m_maxIterations(
            std::stoi(MapHelpers::Get<std::string, std::string>(params, "maxiter", "35")))
      , m_transformationEpsilon(
            std::stof(MapHelpers::Get<std::string, std::string>(params, "transepsilon", "0.005")))
      , m_stepSize(std::stof(MapHelpers::Get<std::string, std::string>(params, "step", "0.01")))
      , m_resolution(
            std::stof(MapHelpers::Get<std::string, std::string>(params, "resolution", "0.1")))
      , m_maxCorrDist(
            std::stof(MapHelpers::Get<std::string, std::string>(params, "maxcorrdist", "0.1")))
      , m_eFitnessEpsilon(
            std::stof(MapHelpers::Get<std::string, std::string>(params, "efitepsilon", "0.1")))
      , m_voxelDownsamplePercentage(
            std::stod(MapHelpers::Get<std::string, std::string>(params, "downsample", "0.1")))
      , m_normalEstimationRadiusSearch(
            std::stod(MapHelpers::Get<std::string, std::string>(params, "normradius", "0.1")))
      , m_featureRadiusSearch(
            std::stod(MapHelpers::Get<std::string, std::string>(params, "featureradius", "0.1")))
      , m_corrRejectMaxIters(std::stoi(
            MapHelpers::Get<std::string, std::string>(params, "corrrejectmaxiters", "100")))
      , m_correRejectInlierThreshold(
            std::stod(MapHelpers::Get<std::string, std::string>(params, "corrrejinlierth", "0.5")))
  {
    LoggingService::Instance().getLogger("ITaskAlignment")->info("Created: {}", *this);
  }

  void ITaskAlignment::setNDTParameters(pcl::NormalDistributionsTransform<PointT, PointT> &ndt)
  {
    ndt.setTransformationEpsilon(m_transformationEpsilon);
    ndt.setStepSize(m_stepSize);
    ndt.setResolution(m_resolution);
    ndt.setMaximumIterations(m_maxIterations);
  }

  void ITaskAlignment::setICPParameters(pcl::IterativeClosestPoint<PointT, PointT> &icp)
  {
    icp.setMaximumIterations(m_maxIterations);
    icp.setMaxCorrespondenceDistance(m_maxCorrDist);
    icp.setTransformationEpsilon(m_transformationEpsilon);
    icp.setEuclideanFitnessEpsilon(m_eFitnessEpsilon);
  }

  void ITaskAlignment::removeOutliers(CloudConstPtr in, CloudPtr out)
  {
    m_logger->trace("Performing statistical outlier point removal");
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setMeanK(m_outlierRemovalMeanK);
    sor.setStddevMulThresh(m_outlierRemovalStdDevMulThr);
    sor.setInputCloud(in);
    sor.filter(*out);
    m_logger->debug("Removed {} outliers ({} points after filtering)", in->size() - out->size(),
                    out->size());
  }

  void ITaskAlignment::downsample(CloudConstPtr in, CloudPtr out)
  {
    m_logger->trace("Downsampling (leaf size: {})", m_voxelDownsamplePercentage);
    pcl::VoxelGrid<PointT> voxelFilter;
    voxelFilter.setLeafSize(m_voxelDownsamplePercentage, m_voxelDownsamplePercentage,
                            m_voxelDownsamplePercentage);
    voxelFilter.setInputCloud(in);
    voxelFilter.filter(*out);
    m_logger->debug("{} points in downsampled cloud", out->size());
  }
}
}