/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskNDTWorldAlignment : public ITaskAlignment
  {
  public:
    TaskNDTWorldAlignment(const boost::filesystem::path &path,
                          std::map<std::string, std::string> &params);

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
