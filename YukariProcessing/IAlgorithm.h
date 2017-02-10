/** @file */

#pragma once

#include <memory>

namespace Yukari
{
namespace Processing
{
  class IAlgorithm
  {
  public:
    IAlgorithm();
    virtual ~IAlgorithm();

    virtual void execute() = 0;
  };

  typedef std::shared_ptr<IAlgorithm> IAlgorithm_sptr;
}
}
