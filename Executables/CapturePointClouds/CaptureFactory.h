/** @file */

#pragma once

#include <YukariCommon/ConfigurationManager.h>

#include "CaptureController.h"

namespace Yukari
{
namespace CaptureApp
{
  class CaptureFactory
  {
  public:
    static CaptureController_sptr Create(Common::ConfigurationManager::Config config);
  };
}
}
