/** @file */

#include "TriggerFactory.h"

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include "PeriodicTrigger.h"
#include "SignalTrigger.h"

using namespace Yukari::Common;

namespace Yukari
{
  namespace Triggers
{
    ITrigger_sptr TriggerFactory::Create(const std::string & fullCommand)
    {
      std::string type;
      std::map<std::string, std::string> params;
      if (!StringParsers::ParseCommand(fullCommand, type, params))
        return nullptr;

      return Create(type, params);
  }

    ITrigger_sptr TriggerFactory::Create(const std::string & type, std::map<std::string, std::string>& parameters)
  {
    std::string lowerType = type;
    StringParsers::CleanString(lowerType);

    ITrigger_sptr trigger;
    if (lowerType == "periodic")
    {
      int seconds = std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "seconds", "10"));
      trigger = std::make_shared<PeriodicTrigger>(std::chrono::seconds(seconds));
    }
    else if (lowerType == "signal")
    {
      int signal = std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "signal", "2"));
      trigger = std::make_shared<SignalTrigger>(signal);
    }

    return trigger;
  }

 }
}
