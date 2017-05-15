/** @file */

#include "ITaskAlignment.h"

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
    icp.setMaxCorrespondenceDistance(m_maxCorrDist);       // 0.5
    icp.setTransformationEpsilon(m_transformationEpsilon); // 1e-6
    icp.setEuclideanFitnessEpsilon(m_eFitnessEpsilon);     // 0.01
  }
}
}