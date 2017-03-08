/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class Add : public Processing::IAlgorithm
  {
  public:
    Add();

  protected:
    virtual void doExecute() override;
  };
}
}
