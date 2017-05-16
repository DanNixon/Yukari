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
        << "maxiter=" << t.m_maxIterations << ","
        << "transepsilon=" << t.m_transformationEpsilon << ","
        << "step=" << t.m_stepSize << ","
        << "resolution=" << t.m_resolution << ","
        << "maxcorr=" << t.m_maxCorrDist << ","
        << "efitepsilon=" << t.m_eFitnessEpsilon << ","
        << "downsample=" << t.m_voxelDownsamplePercentage << ","
        << "normradius=" << t.m_normalEstimationRadiusSearch << ","
        << "featureradius=" << t.m_featureRadiusSearch << ","
        << "corrrejectmaxiters=" << t.m_corrRejectMaxIters << ","
        << "corrrejinlierth=" << t.m_correRejectInlierThreshold << ")";

      return s;
    }

  protected:
    void setNDTParameters(pcl::NormalDistributionsTransform<PointT, PointT> &ndt);
    void setICPParameters(pcl::IterativeClosestPoint<PointT, PointT> &icp);

    virtual void doAlignment(Task t) = 0;

  protected:
    /* Outlier removal parameters */
    bool m_outlierRemoval;
    int m_outlierRemovalMeanK;
    double m_outlierRemovalStdDevMulThr;

    /* Common parameters */
    int m_maxIterations;
    double m_transformationEpsilon;

    /* NDT parameters */
    double m_stepSize;
    double m_resolution;

    /* ICP parameters */
    double m_maxCorrDist;
    double m_eFitnessEpsilon;

    /* Voxel filter parameters */
    double m_voxelDownsamplePercentage;

    /* Normal estimation parameters */
    double m_normalEstimationRadiusSearch;

    /* Feature estimation parameters */
    double m_featureRadiusSearch;

    /* Correspondence rejection parameters */
    int m_corrRejectMaxIters;
    double m_correRejectInlierThreshold;
  };
}
}
