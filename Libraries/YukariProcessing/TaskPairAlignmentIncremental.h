/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskPairAlignmentIncremental : public ITaskAlignment
  {
  public:
    TaskPairAlignmentIncremental(const boost::filesystem::path &path,
                                 std::map<std::string, std::string> &params);

    virtual int process(Task t) override;

  private:
    Common::LoggingService::Logger m_logger;

    bool m_saveTransforms;
    bool m_saveClouds;

    CloudPtr m_previousCloud;
    Eigen::Matrix4f m_previousCloudWorldTransform;
  };
}
}
