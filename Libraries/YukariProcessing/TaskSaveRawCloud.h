/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskSaveRawCloud : public IFrameProcessingTask
  {
  public:
    TaskSaveRawCloud(const boost::filesystem::path &path, bool transform = false);

    virtual int process(Task t) override;

  private:
    Common::LoggingService::Logger m_logger;

    bool m_transform;
  };
}
}
