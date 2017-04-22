/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>

#include "CloudOperations.h"
#include "SpatialOperations.h"

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE>
  class TaskNDTWorldAlignment : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    TaskNDTWorldAlignment(const boost::filesystem::path &path)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskNDTWorldAlignment"))
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
      pcl::transformPointCloud(*t.cloud, *inputCloud, t.imuFrame->position().toEigen(),
                               Processing::SpatialOperations::RotateQuaternionForCloud(
                                   t.imuFrame->orientation().toEigen()));

      if (!m_worldCloud)
      {
        /* If this is the first recored cloud simply set it as he "world" cloud */
        m_worldCloud = CloudPtr(new Cloud(*inputCloud));
      }
      else
      {
        /* Otherwise alignment is required */

        /* Downsample the cloud for alignment */
        auto filteredInputCloud =
            Processing::CloudOperations<POINT_TYPE>::DownsampleVoxelFilter(inputCloud);

        /* Perform alignment */
        pcl::NormalDistributionsTransform<POINT_TYPE, POINT_TYPE> ndt;

        ndt.setTransformationEpsilon(0.005);
        ndt.setStepSize(0.01);
        ndt.setResolution(0.1);

        ndt.setMaximumIterations(35);

        ndt.setInputSource(filteredInputCloud);
        ndt.setInputTarget(m_worldCloud);

        /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
        CloudPtr transformedInputCloud(new Cloud());
        ndt.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

        if (ndt.hasConverged())
          m_logger->info("Convergence reached");
        else
          m_logger->warn("Convergence not reached");
        m_logger->info("Normal Distributions Transform score: {}", ndt.getFitnessScore());

        /* Translate full input cloud */
        pcl::transformPointCloud(*inputCloud, *transformedInputCloud, ndt.getFinalTransformation());

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
