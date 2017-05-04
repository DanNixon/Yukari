/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <boost/filesystem/path.hpp>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskSaveRawIMUFrame : public IFrameProcessingTask
  {
  public:
    TaskSaveRawIMUFrame(const boost::filesystem::path &path);

    virtual int process(Task t) override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
