/** @file */

#include "StringParsers.h"

#include <boost/regex.hpp>

namespace Yukari
{
namespace Common
{
  bool StringParsers::ParseCommand(const std::string &input, std::string &command,
                                   std::map<std::string, std::string> &parameters)
  {
    auto log = LoggingService::Instance().getLogger("StringParsers");

    boost::regex expression("(\\w+)\\s*(\\(\\s*(.*)\\s*\\))?");

    boost::cmatch what;
    if (boost::regex_match(input.c_str(), what, expression))
    {
      log->trace("Got parameter name: \"{}\", parameter list: \"{}\"", what[1], what[3]);
      command = what[1];
      return (what[3].length() == 0) ? true : ParseParameterList(what[3], parameters);
    }
    else
    {
      log->error("Could not parse regex for command name");
      return false;
    }
  }

  bool StringParsers::ParseParameterList(const std::string &input,
                                         std::map<std::string, std::string> &parameters)
  {
    auto log = LoggingService::Instance().getLogger("StringParsers");

    boost::regex expression("[\\s,]*([\\w\\.]+)\\s*\\=\\s*([\\w\\.\\/\\\\\\s]+)[\\s,]*");

    auto start = input.cbegin();
    auto end = input.cend();

    boost::match_results<std::string::const_iterator> what;
    boost::match_flag_type flags = boost::match_default;

    while (boost::regex_search(start, end, what, expression, flags))
    {
      log->trace("Got parameter: \"{}\"=\"{}\"", what[1], what[2]);
      parameters[what[1]] = what[2];

      start = what[0].second;

      flags |= boost::match_prev_avail;
      flags |= boost::match_not_bob;
    }

    return !parameters.empty();
  }
}
}
