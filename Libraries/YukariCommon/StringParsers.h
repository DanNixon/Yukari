/** @file */

#pragma once

#include <string>
#include <map>

#include "LoggingService.h"

namespace Yukari
{
  namespace Common
  {
    class StringParsers
    {
    public:
      static bool ParseCommand(const std::string & input, std::string & command, std::map<std::string, std::string> & parameters);
      static bool ParseParameterList(const std::string & input, std::map<std::string, std::string> & parameters);
    };
  }
}