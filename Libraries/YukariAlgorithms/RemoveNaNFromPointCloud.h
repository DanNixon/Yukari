/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class RemoveNaNFromPointCloud : public Processing::IAlgorithm
  {
  public:
    RemoveNaNFromPointCloud();

  protected:
    virtual void doExecute() override;
  };
}
}
