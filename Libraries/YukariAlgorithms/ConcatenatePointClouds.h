/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class ConcatenatePointClouds : public Processing::IAlgorithm
  {
  public:
    ConcatenatePointClouds();

    virtual void execute() override;
  };
}
}
