/** @file */

#pragma once

#include "ITaskIncrementalAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskPairIncrementalAlignment : public ITaskIncrementalAlignment
  {
  public:
    TaskPairIncrementalAlignment(const boost::filesystem::path &path,
                                 std::map<std::string, std::string> &params);

  protected:
    virtual void doAlignment(Task t) override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}