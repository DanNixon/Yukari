/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskAppendTransformedClouds : public IFrameProcessingTask
  {
  public:
    TaskAppendTransformedClouds(const boost::filesystem::path &path);

    inline CloudPtr worldCloud()
    {
      return m_worldCloud;
    }

    virtual int process(Task t) override;
    virtual int onStop() override;

  private:
    Common::LoggingService::Logger m_logger;

    CloudPtr m_worldCloud;
  };
}
}
