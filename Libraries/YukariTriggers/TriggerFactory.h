/** @file */

#pragma once

#include <string>
#include <map>

#include "ITrigger.h"

namespace Yukari
{
namespace Triggers
{
  class TriggerFactory
  {
  public:
    static ITrigger_sptr Create(const std::string &fullCommand);
    static ITrigger_sptr Create(const std::string &type, std::map<std::string, std::string> & parameters);
  };
}
}
