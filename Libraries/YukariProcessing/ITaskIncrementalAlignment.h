/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class ITaskIncrementalAlignment : public ITaskAlignment
  {
  public:
    ITaskIncrementalAlignment(const boost::filesystem::path &path,
                              std::map<std::string, std::string> &params);

    virtual int process(Task t) override;

  protected:
    virtual void doAlignment(Task t) = 0;

  protected:
    bool m_saveTransforms;
    bool m_saveClouds;

    CloudPtr m_previousCloud;
    Eigen::Matrix4f m_previousCloudWorldTransform;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
