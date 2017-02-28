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

  protected:
    virtual void doExecute() override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
