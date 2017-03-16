/** @file */

#include <iostream>

#include <boost/program_options.hpp>
#include <boost/qvm/all.hpp>
#include <serial/serial.h>

#include <YukariCaptureTriggers/ITrigger.h>
#include <YukariCommon/LoggingService.h>

using namespace Yukari::CaptureTriggers;
using namespace Yukari::Common;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  auto logger = LoggingService::GetLogger("main");

  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
	  ("help", "Show brief usage message")
	  ("trigger", po::value<std::string>()->default_value(""), "Name of trigger to add");
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

  /* Start trigger test */
  const std::string mode = args["trigger"].as<std::string>();
  /* if () */
  /* ; */
  /* else */
  return 2;
}
