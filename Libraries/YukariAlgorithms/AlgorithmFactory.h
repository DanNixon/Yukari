/** @file */

#pragma once

#include <string>

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Algorithms
{
  class AlgorithmFactory
  {
  public:
    static Processing::IAlgorithm_sptr Create(const std::string &name);
  };
}
}
