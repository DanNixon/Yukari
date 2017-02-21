/** @file */

#include <iostream>

#include <boost/program_options.hpp>
#include <boost/qvm/all.hpp>

#include <YukariCommon/ConfigurationManager.h>
#include <YukariCommon/LoggingService.h>
#include <YukariMaths/Quaternion.h>

#include "CaptureCLI.h"
#include "CaptureFactory.h"

using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::CaptureApp;
using namespace boost::qvm;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
	  ("help", "Show brief usage message")
	  ("cli", "Start with CLI enabled")
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
    config = ConfigurationManager::LoadFromAppDataDirectory("yukari", "capture_config.json");

  /* Configure logging */
  LoggingService::Configure(config);
  auto logger = LoggingService::GetLogger("YukariCaptureApp");

  /* Create capture controller */
  CaptureController_sptr captureController = CaptureFactory::Create(config);
  if (!captureController)
  {
    logger->error("Could not create capture controller!");
    return 2;
  }

  /* Add CLI if required */
  std::shared_ptr<CaptureCLI> cli;
  if (args.count("cli") || config.get<bool>("capture.cli", false))
  {
    logger->info("Adding CLI");
    cli = std::make_shared<CaptureCLI>(std::cin, std::cout);
    cli->init(captureController);
    cli->runAsync();
  }

  logger->info("Capture controller: {}", *captureController);
  LoggingService::Flush();

  /* Run capture */
  int exitCode = captureController->run();

  /* Exit */
  if (cli)
  {
    cli->exit();
    cli->join();
  }
  LoggingService::Flush();
  return exitCode;
}
