/** @file */

#pragma once

#include "ITaskNDTAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>

#include "CloudOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE>
  class TaskNDTIncrementalAlignment : public ITaskNDTAlignment<POINT_TYPE>
  {
  public:
    TaskNDTIncrementalAlignment(const boost::filesystem::path &path,
                                std::map<std::string, std::string> &params)
        : ITaskNDTAlignment(path, params)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskNDTIncrementalAlignment"))
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

    virtual int process(Task t) override
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
        auto filteredInputCloud = Processing::CloudOperations<POINT_TYPE>::DownsampleVoxelFilter(
            t.cloud, m_voxelDownsamplePercentage);

        /* Perform alignment */
        pcl::NormalDistributionsTransform<POINT_TYPE, POINT_TYPE> ndt;
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
      pcl::transformPointCloud<POINT_TYPE>(*m_previousCloud, *m_previousCloud,
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
        IMUFrame transformFrame(m_previousCloudWorldTransform);
        boost::filesystem::path imuFilename = m_outputDirectory / (frameNoStr + "_transform.txt");
        transformFrame.save(imuFilename);
      }

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;

    bool m_saveTransforms;
    bool m_saveClouds;

    CloudPtr m_previousCloud;
    Eigen::Matrix4f m_previousCloudWorldTransform;
  };
}
}
