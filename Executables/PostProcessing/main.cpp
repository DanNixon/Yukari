/** @file */

#include <iostream>

#include <boost/program_options.hpp>
#include <pcl/point_types.h>

#include <YukariCommon/LoggingService.h>
#include <YukariProcessing/IFrameProcessingTask.h>

using namespace Yukari::Common;
using namespace Yukari::Processing;
namespace po = boost::program_options;

typedef pcl::PointXYZRGBA PointType;
typedef IFrameProcessingTask<PointType>::Ptr ProcessingTaskPtr;

int main(int argc, char **argv)
{
  auto logger = LoggingService::Instance().getLogger("YukariCaptureApp");

  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
    ("help", "Show brief usage message")
    ("loglevel", po::value<std::string>()->default_value("debug"), "Global log level")
    ("in", po::value<std::string>()->default_value("."), "Input directory")
    ("out", po::value<std::string>()->default_value("."), "Output directory")
    ("process", po::value<std::vector<std::string>>()->multitoken(), "Add processing stage");
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
  LoggingService::Instance().configure(args);

  /* TODO */

  /* Exit */
  LoggingService::Instance().flush();
  return 0;
}
