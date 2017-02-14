/** @file */

#pragma once

#include <memory>

namespace Yukari
{
namespace CaptureApp
{
  class ITrigger
  {
  };

  typedef std::shared_ptr<ITrigger> ITrigger_sptr;
}
}
