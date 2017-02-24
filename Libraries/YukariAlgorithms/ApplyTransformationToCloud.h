/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class ApplyTransformationToCloud : public Processing::IAlgorithm
  {
  public:
    ApplyTransformationToCloud();

    virtual void execute() override;
  };
}
}
