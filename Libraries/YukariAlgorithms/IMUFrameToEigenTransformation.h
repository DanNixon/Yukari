/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class IMUFrameToEigenTransformation : public Processing::IAlgorithm
  {
  public:
    IMUFrameToEigenTransformation();

    virtual void execute() override;
  };
}
}
