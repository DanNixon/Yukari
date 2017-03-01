/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class GenerateMeshFromPointCloud : public Processing::IAlgorithm
  {
  public:
    GenerateMeshFromPointCloud();

  protected:
    virtual void doExecute() override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
