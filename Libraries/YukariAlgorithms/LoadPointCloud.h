/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class LoadPointCloud : public Processing::IAlgorithm
  {
  public:
    LoadPointCloud();

    virtual void execute() override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
