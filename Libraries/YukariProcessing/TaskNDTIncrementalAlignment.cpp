/** @file */

#include "TaskNDTIncrementalAlignment.h"

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include "CloudOperations.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskNDTIncrementalAlignment::TaskNDTIncrementalAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskNDTIncrementalAlignment"))
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

  int TaskNDTIncrementalAlignment::process(Task t)
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
      /* Downsample the input cloud for alignment */
      auto filteredInputCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
          t.cloud, m_voxelDownsamplePercentage);

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
      m_previousCloudWorldTransform = ndt.getFinalTransformation();
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