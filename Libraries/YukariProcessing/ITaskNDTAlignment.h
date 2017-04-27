/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class ITaskNDTAlignment : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    ITaskNDTAlignment(const boost::filesystem::path &path)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("ITaskNDTAlignment"))
    {
    }

  protected:
    void setNDTParameters(pcl::NormalDistributionsTransform<POINT_TYPE, POINT_TYPE> &ndt)
    {
      ndt.setTransformationEpsilon(0.005);
      ndt.setStepSize(0.01);
      ndt.setResolution(0.1);

      ndt.setMaximumIterations(35);
    }

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
