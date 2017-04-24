/** @file */

#pragma once

#include <boost/program_options.hpp>

#include "CaptureController.h"

namespace Yukari
{
namespace Capture
{
  class CaptureFactory
  {
  public:
    static CaptureController::Ptr Create(boost::program_options::variables_map config);
  };
}
}
