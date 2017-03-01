/** @file */

#include <iostream>

#include <boost/program_options.hpp>

#include <YukariCommon/ConfigurationManager.h>
#include <YukariCommon/LoggingService.h>

#include "ProcessingCLI.h"

using namespace Yukari::Common;
using namespace Yukari::ProcessingApp;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
    ("help", "Show brief usage message")
    ("config", po::value<std::string>(), "Configuration file to use");
  // clang-format on

  /* Parse command line args */
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), args);
  }
  catch (po::error const &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }

  /* Show usage */
  if (args.count("help"))
  {
    std::cout << desc << "\n";
    return 1;
  }

  /* Load configuration */
  ConfigurationManager::Config config;
  if (args.count("config"))
    config = ConfigurationManager::Load(args["config"].as<std::string>());
  else
    config = ConfigurationManager::LoadFromAppDataDirectory("yukari", "process_config.json");

  /* Configure logging */
  LoggingService::Configure(config);
  auto logger = LoggingService::GetLogger("YukariProcessingApp");

  /* Start CLI */
  ProcessingCLI cli(std::cin, std::cout);
  int exitCode = cli.run();

  /* Exit */
  LoggingService::Flush();
  return exitCode;
}
