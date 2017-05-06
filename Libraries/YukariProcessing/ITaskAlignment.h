/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <map>
#include <ostream>
#include <string>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace Yukari
{
namespace Processing
{
  class ITaskAlignment : public IFrameProcessingTask
  {
  public:
    ITaskAlignment(const boost::filesystem::path &path, std::map<std::string, std::string> &params);

    friend std::ostream &operator<<(std::ostream &s, const ITaskAlignment &t)
    {
      s << "ITaskAlignment("
        << "epsilon=" << t.m_transformationEpsilon << ","
        << "step=" << t.m_stepSize << ","
        << "resolution=" << t.m_resolution << ","
        << "maxiter=" << t.m_maxIterations << ","
        << "downsample=" << t.m_voxelDownsamplePercentage << ")";

      return s;
    }

  protected:
    void setNDTParameters(pcl::NormalDistributionsTransform<PointT, PointT> &ndt);
    void setICPParameters(pcl::IterativeClosestPoint<PointT, PointT> &icp);

    virtual void doAlignment(Task t) = 0;

  protected:
    /* NDT parameters */
    double m_transformationEpsilon;
    double m_stepSize;
    double m_resolution;
    int m_maxIterations;

    /* ICP parameters */
    /* TODO */

    /* Voxel filter parameters */
    double m_voxelDownsamplePercentage;
  };
}
}
