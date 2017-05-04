/** @file */

#pragma once

#include <map>
#include <string>

#include "IFrameProcessingTask.h"

namespace Yukari
{
namespace Processing
{
  class FrameProcessingTaskFactory
  {
  public:
    static IFrameProcessingTask::Ptr Create(const std::string &fullCommand,
                                            const boost::filesystem::path &rootOutputDirectory);

    static IFrameProcessingTask::Ptr Create(const std::string &type,
                                            std::map<std::string, std::string> &parameters,
                                            const boost::filesystem::path &rootOutputDirectory);
  };
}
}
