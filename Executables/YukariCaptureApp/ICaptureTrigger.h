/** @file */

#pragma once

#include <memory>

namespace Yukari
{
namespace CaptureApp
{
  class ICaptureTrigger
  {
  };

  typedef std::shared_ptr<ICaptureTrigger> ICaptureTrigger_sptr;
}
}
