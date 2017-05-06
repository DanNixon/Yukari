/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class ITaskWorldAlignment : public ITaskAlignment
  {
  public:
    ITaskWorldAlignment(const boost::filesystem::path &path,
                          std::map<std::string, std::string> &params);

    virtual int process(Task t) override;
    virtual int onStop() override;

  protected:
    virtual void doAlignment(Task t) = 0;

  protected:
    CloudPtr m_worldCloud;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
