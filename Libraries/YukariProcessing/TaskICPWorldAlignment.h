/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <YukariCommon/LoggingService.h>

#include "CloudOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class TaskICPWorldAlignment : public ITaskAlignment<POINT_TYPE>
  {
  public:
    TaskICPWorldAlignment(const boost::filesystem::path &path,
                          std::map<std::string, std::string> &params)
        : ITaskAlignment(path, params)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskICPWorldAlignment"))
        , m_worldCloud()
    {
    }

    inline CloudPtr worldCloud()
    {
      return m_worldCloud;
    }

    virtual int process(Task t) override
    {
      if (!(t.cloud && t.imuFrame))
      {
        m_logger->error("Do not have both cloud and IMU frame");
        return 1;
      }

      CloudPtr inputCloud(new Cloud());

      /* Transform cloud */
      m_logger->trace("Transforming cloud by IMU");
      pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->toCloudTransform());

      if (!m_worldCloud)
      {
        /* If this is the first recored cloud simply set it as he "world" cloud */
        m_worldCloud = CloudPtr(new Cloud(*inputCloud));
      }
      else
      {
        /* Otherwise alignment is required */

        /* Downsample the input cloud for alignment */
        auto filteredInputCloud = Processing::CloudOperations<POINT_TYPE>::DownsampleVoxelFilter(
            inputCloud, m_voxelDownsamplePercentage);

        /* Perform alignment */
        pcl::IterativeClosestPoint<POINT_TYPE, POINT_TYPE> icp;
        setICPParameters(icp);

        icp.setInputSource(filteredInputCloud);
        icp.setInputTarget(m_worldCloud);

        /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
        CloudPtr transformedInputCloud(new Cloud());
        icp.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

        if (icp.hasConverged())
          m_logger->debug("Convergence reached");
        else
          m_logger->warn("Convergence not reached");
        m_logger->debug("After {} iterations", icp.getFinalNumIteration());
        m_logger->debug("Fitness score: {}", icp.getFitnessScore());

        /* Translate full input cloud */
        pcl::transformPointCloud(*inputCloud, *transformedInputCloud, icp.getFinalTransformation());

        /* Add translated cloud to world cloud */
        *m_worldCloud += *transformedInputCloud;
      }

      return 0;
    }

    virtual int onStop() override
    {
      if (!m_worldCloud)
      {
        m_logger->warn("No world cloud, nothing saved");
        return 1;
      }

      /* Generate filename */
      boost::filesystem::path cloudFilename = m_outputDirectory / "world_cloud.pcd";

      /* Save world cloud */
      m_logger->trace("Saving world point cloud: {}", cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_worldCloud);

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;

    CloudPtr m_worldCloud;
  };
}
}
