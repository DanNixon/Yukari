/** @file */

#include <iostream>

#include <boost/program_options.hpp>

#include <YukariCommon/LoggingService.h>

#include "CaptureCLI.h"
#include "CaptureFactory.h"

using namespace Yukari::Common;
using namespace Yukari::CaptureApp;
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
    ("dir", po::value<std::string>()->default_value("."), "Root output directory")
    ("cloudgrabber", po::value<std::string>()->default_value("dummy"), "Cloud grabber to use")
    ("imugrabber", po::value<std::string>()->default_value("dummy"), "IMU grabber to use")
    ("capturetrigger", po::value<std::string>()->default_value("periodic(seconds=5)"), "Trigger for a single frame");
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

  /* Configure logging */
  auto logger = LoggingService::Instance().getLogger("YukariCaptureApp");

  /* Create capture controller */
  CaptureController_sptr captureController = CaptureFactory::Create(args);
  if (!captureController)
  {
    logger->error("Could not create capture controller!");
    return 2;
  }

  /* Add CLI if required */
  std::shared_ptr<CaptureCLI> cli;
  if (args.count("cli"))
  {
    logger->info("Adding CLI");
    cli = std::make_shared<CaptureCLI>(std::cin, std::cout);
    cli->init(captureController);
    cli->runAsync();
  }

  logger->info("Capture controller: {}", *captureController);
  LoggingService::Instance().flush();

  /* Run capture */
  int exitCode = captureController->run();

  /* Exit */
  if (cli)
  {
    cli->exit();
    cli->join();
  }
  LoggingService::Instance().flush();
  return exitCode;
}
