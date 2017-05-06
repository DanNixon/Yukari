/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskDownsampleCloud : public ITaskAlignment
  {
  public:
    TaskDownsampleCloud(const boost::filesystem::path &path,
                        std::map<std::string, std::string> &params);

    virtual int process(Task t) override;

  protected:
    virtual void doAlignment(Task t) override
    {
      /* Nothing to do */
    }

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
