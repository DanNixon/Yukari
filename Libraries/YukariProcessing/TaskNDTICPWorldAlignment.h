/** @file */

#pragma once

#include "ITaskWorldAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskNDTICPWorldAlignment : public ITaskWorldAlignment
  {
  public:
    TaskNDTICPWorldAlignment(const boost::filesystem::path &path,
                             std::map<std::string, std::string> &params);

  protected:
    virtual void doAlignment(Task t) override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
