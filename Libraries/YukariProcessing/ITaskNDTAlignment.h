/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <map>
#include <ostream>
#include <string>

#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class ITaskNDTAlignment : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    ITaskNDTAlignment(const boost::filesystem::path &path,
                      std::map<std::string, std::string> &params)
        : IFrameProcessingTask(path)
        , m_transformationEpsilon(
              std::stof(MapHelpers::Get<std::string, std::string>(params, "epsilon", "0.01")))
        , m_stepSize(std::stof(MapHelpers::Get<std::string, std::string>(params, "step", "0.1")))
        , m_resolution(
              std::stof(MapHelpers::Get<std::string, std::string>(params, "resolution", "0.01")))
        , m_maxIterations(
              std::stoi(MapHelpers::Get<std::string, std::string>(params, "maxiter", "50")))
        , m_voxelDownsamplePercentage(
              std::stod(MapHelpers::Get<std::string, std::string>(params, "downsample", "0.01")))
    {
      Common::LoggingService::Instance().getLogger("ITaskNDTAlignment")->info("Created: {}", *this);
    }

    friend std::ostream &operator<<(std::ostream &s, const ITaskNDTAlignment<POINT_TYPE> &t)
    {
      s << "ITaskNDTAlignment("
        << "epsilon=" << t.m_transformationEpsilon << ","
        << "step=" << t.m_stepSize << ","
        << "resolution=" << t.m_resolution << ","
        << "maxiter=" << t.m_maxIterations << ","
        << "downsample=" << t.m_voxelDownsamplePercentage << ")";

      return s;
    }

  protected:
    void setNDTParameters(pcl::NormalDistributionsTransform<POINT_TYPE, POINT_TYPE> &ndt)
    {
      ndt.setTransformationEpsilon(m_transformationEpsilon);
      ndt.setStepSize(m_stepSize);
      ndt.setResolution(m_resolution);
      ndt.setMaximumIterations(m_maxIterations);
    }

  protected:
    double m_transformationEpsilon;
    double m_stepSize;
    double m_resolution;
    int m_maxIterations;

    double m_voxelDownsamplePercentage;
  };
}
}
