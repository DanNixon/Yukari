/** @file */

#pragma once

#include <boost/program_options.hpp>

#include "CaptureController.h"

namespace Yukari
{
namespace CaptureApp
{
  class CaptureFactory
  {
  public:
    static CaptureController_sptr Create(boost::program_options::variables_map config);
  };
}
}
