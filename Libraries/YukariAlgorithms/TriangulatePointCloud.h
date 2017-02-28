/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class TriangulatePointCloud : public Processing::IAlgorithm
  {
  public:
    TriangulatePointCloud();

  protected:
    virtual void doExecute() override;
  };
}
}
