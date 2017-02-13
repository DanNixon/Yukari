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

  typedef std::shared_ptr<IIMUGrabber> ICaptureTrigger_sptr;
}
}
