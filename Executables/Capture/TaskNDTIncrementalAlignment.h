/** @file */

#pragma once

#include "IPostCaptureTask.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>
#include <YukariProcessing/CloudOperations.h>
#include <YukariProcessing/SpatialOperations.h>

#include "Types.h"

namespace Yukari
{
namespace CaptureApp
{
  class TaskNDTIncrementalAlignment : public IPostCaptureTask
  {
  public:
    TaskNDTIncrementalAlignment(const boost::filesystem::path &path)
        : IPostCaptureTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("NDTIncrementalAlignment"))
        , m_worldCloud()
    {
    }

    inline CloudPtr worldCloud()
    {
      return m_worldCloud;
    }

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) override
    {
      if (!(cloud && imuFrame))
      {
        m_logger->error("Do not have both cloud and IMU frame");
        return 1;
      }

      CloudPtr inputCloud(new Cloud());

      /* Transform cloud */
      m_logger->trace("Transforming cloud by IMU");
      pcl::transformPointCloud(*cloud, *inputCloud, imuFrame->position().toEigen(),
                               Processing::SpatialOperations::RotateQuaternionForCloud(
                                   imuFrame->orientation().toEigen()));

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
            Processing::CloudOperations<PointType>::DownsampleVoxelFilter(inputCloud);

        /* Perform alignment */
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;

        ndt.setTransformationEpsilon(0.005);
        ndt.setStepSize(0.01);
        ndt.setResolution(0.1);

        ndt.setMaximumIterations(35);

        ndt.setInputSource(filteredInputCloud);
        ndt.setInputTarget(m_worldCloud);

        /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedInputCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        ndt.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

        if (ndt.hasConverged())
          m_logger->info("Convergence reached");
        else
          m_logger->warn("Convergence not reached");
        m_logger->info("Normal Distributions Transform score: {}", ndt.getFitnessScore());

        /* Translate full input cloud */
        pcl::transformPointCloud(*inputCloud, *transformedInputCloud, ndt.getFinalTransformation());

        /* Add translated cloud to world cloud */
        // TODO
      }

      return 0;
    }

    virtual int onStop() override
    {
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
