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
    static Processing::IAlgorithm_sptr Create(std::string name);

    static Processing::IAlgorithm_sptr Create(const std::string &name,
                                              std::vector<std::string>::const_iterator begin,
                                              std::vector<std::string>::const_iterator end);
  };
}
}
