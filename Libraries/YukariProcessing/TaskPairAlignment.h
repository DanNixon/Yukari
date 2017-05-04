/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <YukariCommon/LoggingService.h>

#include "CloudOperations.h"

/* A bit of a hacky way to get the correct OMP instantiation without recompiling PCL */
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/impl/instantiate.hpp>
PCL_INSTANTIATE_PRODUCT(NormalEstimationOMP, ((pcl::PointXYZRGBA))((pcl::PointNormal)))

namespace Yukari
{
namespace Processing
{
  class RegistrationPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
  {
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

  public:
    RegistrationPointRepresentation()
    {
      nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.curvature;
    }
  };

  class TaskPairAlignment : public ITaskAlignment
  {
  public:
    TaskPairAlignment(const boost::filesystem::path &path,
                      std::map<std::string, std::string> &params)
        : ITaskAlignment(path, params)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskPairAlignment"))
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

        /* Downsample the input and world cloud for alignment */
        auto filteredInputCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
            inputCloud, m_voxelDownsamplePercentage);
        auto filteredWorldCloud = Processing::CloudOperations<PointT>::DownsampleVoxelFilter(
            m_worldCloud, m_voxelDownsamplePercentage);

        /* Compute normals and curvature */
        pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals(
            new pcl::PointCloud<pcl::PointNormal>());
        pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals(
            new pcl::PointCloud<pcl::PointNormal>());

        pcl::NormalEstimationOMP<PointT, pcl::PointNormal> normalEst;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        normalEst.setSearchMethod(tree);
        normalEst.setRadiusSearch(0.1);

        normalEst.setInputCloud(filteredInputCloud);
        normalEst.compute(*sourceNormals);

        normalEst.setInputCloud(filteredWorldCloud);
        normalEst.compute(*targetNormals);

        /* Init registration */
        RegistrationPointRepresentation pr;
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        pr.setRescaleValues(alpha);

        pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
        reg.setMaximumIterations(50);
        reg.setTransformationEpsilon(1e-6);
        reg.setMaxCorrespondenceDistance(0.001);

        reg.setPointRepresentation(boost::make_shared<const RegistrationPointRepresentation>(pr));

        reg.setInputSource(sourceNormals);
        reg.setInputTarget(targetNormals);

        /* Align */
        pcl::PointCloud<pcl::PointNormal>::Ptr regResult(new pcl::PointCloud<pcl::PointNormal>());
        reg.align(*regResult);

        if (reg.hasConverged())
          m_logger->debug("Convergence reached");
        else
          m_logger->warn("Convergence not reached");
        m_logger->debug("Fitness score: {}", reg.getFitnessScore());

        Eigen::Matrix4f transformNormal = reg.getFinalTransformation();
        Eigen::Matrix4f transform = transformNormal.inverse();
        m_logger->debug("Final transform: {}", transform);

        /* Translate full input cloud */
        pcl::transformPointCloud(*inputCloud, *inputCloud, transform);

        /* Add translated cloud to world cloud */
        *m_worldCloud += *inputCloud;
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
