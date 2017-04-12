/** @file */

#include <iostream>

#include <boost/program_options.hpp>
#include <boost/qvm/all.hpp>
#include <serial/serial.h>

#include <YukariCaptureTriggers/ITrigger.h>
#include <YukariCaptureTriggers/SignalTrigger.h>
#include <YukariCaptureTriggers/TimelapseCaptureTrigger.h>
#include <YukariCommon/LoggingService.h>

using namespace Yukari::CaptureTriggers;
using namespace Yukari::Common;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  auto logger = LoggingService::Instance().getLogger("main");

  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
    ("help", "Show brief usage message")
    ("trigger", po::value<std::string>()->default_value(""), "Name of trigger to add")
    ("seconds", po::value<int>()->default_value(1), "Timelapse duration in seconds")
    ("signal", po::value<int>()->default_value(2), "POSIX signal number");
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

  /* Get trigger */
  ITrigger_sptr trigger;
  const std::string triggerName = args["trigger"].as<std::string>();
  if (triggerName == "timelapse")
    trigger =
        std::make_shared<TimelapseCaptureTrigger>(std::chrono::seconds(args["seconds"].as<int>()));
  else if (triggerName == "signal")
    trigger = std::make_shared<SignalTrigger>(args["signal"].as<int>());

  if (!trigger)
  {
    logger->error("No trigger!");
    return 2;
  }

  /* Run trigger test */
  trigger->setHandler([&logger]() { logger->info("Triggered."); });
  trigger->enable();
  while (true)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return 0;
}
