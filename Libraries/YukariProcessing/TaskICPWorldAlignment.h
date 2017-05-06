/** @file */

#pragma once

#include "ITaskWorldAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskICPWorldAlignment : public ITaskWorldAlignment
  {
  public:
    TaskICPWorldAlignment(const boost::filesystem::path &path,
                          std::map<std::string, std::string> &params);

  protected:
    virtual void doAlignment(Task t) override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
