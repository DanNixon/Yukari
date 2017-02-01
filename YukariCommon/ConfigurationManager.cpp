/** @file */

#include "ConfigurationManager.h"

#include "LoggingService.h"

#include <boost/property_tree/json_parser.hpp>

namespace Yukari
{
namespace Common
{
  ConfigurationManager::Config ConfigurationManager::Load(const std::string &filename)
  {
    auto logger = LoggingService::GetLogger("ConfigurationManager");

    boost::property_tree::ptree pt;
    try
    {
      logger->debug("Reading configuration file \"{}\"", filename);
      boost::property_tree::read_json(filename, pt);
    }
    catch (boost::property_tree::json_parser_error &e)
    {
      logger->warn("Failed to read configuration file \"{}\": {}", filename, e.what());
    }

    return pt;
  }
}
}
