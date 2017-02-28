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

  protected:
    virtual void doExecute() override;
  };
}
}
