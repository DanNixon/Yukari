/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class SaveVTKMesh : public Processing::IAlgorithm
  {
  public:
    SaveVTKMesh();

  protected:
    virtual void doExecute() override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
