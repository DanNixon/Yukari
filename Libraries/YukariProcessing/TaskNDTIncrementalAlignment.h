/** @file */

#pragma once

#include "IFrameProcessingTask.h"

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
  class TaskNDTIncrementalAlignment : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    TaskNDTIncrementalAlignment(const boost::filesystem::path &path)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskNDTIncrementalAlignment"))
        , m_previousCloud()
    {
    }

    virtual int process(Task t) override
    {
      if (!(t.cloud && t.imuFrame))
      {
        m_logger->error("Do not have both cloud and IMU frame");
        return 1;
      }

      if (!m_previousCloud)
      {
        /* If no previous cloud exists set it */
        m_previousCloud = t.cloud;
        // TODO

        /* Save raw IMU transform */
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << t.frameNumber;
        boost::filesystem::path imuFilename = m_outputDirectory / (ss.str() + "_transform.txt");
        t.imuFrame->save(imuFilename);
      }
      else
      {
        /* Downsample the input cloud for alignment */
        auto filteredInputCloud =
            Processing::CloudOperations<POINT_TYPE>::DownsampleVoxelFilter(inputCloud);

        /* Perform alignment */
        pcl::NormalDistributionsTransform<POINT_TYPE, POINT_TYPE> ndt;

        ndt.setTransformationEpsilon(0.005);
        ndt.setStepSize(0.01);
        ndt.setResolution(0.1);

        ndt.setMaximumIterations(35);

        ndt.setInputSource(filteredInputCloud);
        ndt.setInputTarget(m_previousCloud);

        /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
        CloudPtr transformedInputCloud(new Cloud());
        ndt.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

        if (ndt.hasConverged())
          m_logger->info("Convergence reached");
        else
          m_logger->warn("Convergence not reached");
        m_logger->info("Normal Distributions Transform score: {}", ndt.getFitnessScore());

        /* Save final transformation between the new cloud and previous cloud */
        Eigen::Matrix4f transform = ndt.getFinalTransformation();
        IMUFrame transformFrame(transform);

        // TODO

        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << t.frameNumber;
        boost::filesystem::path imuFilename = m_outputDirectory / (ss.str() + "_transform.txt");
        transformFrame->save(imuFilename);

        /* Set new previous frame */
        m_previousCloud = inputCloud;
      }

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;

    CloudPtr m_previousCloud;
    Eigen::Matrix4f m_previousCloudWorldTransform;
  };
}
}
