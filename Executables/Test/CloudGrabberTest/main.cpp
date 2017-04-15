/** @file */

#include <boost/program_options.hpp>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/openni2_grabber.h>

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCommon/LoggingService.h>
#include <YukariCommon/StringParsers.h>
#include <YukariIMU/IMUGrabberFactory.h>

#include "CloudGrabberVisualisation.h"

namespace po = boost::program_options;
using namespace Yukari::CloudCapture;
using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Maths;

int main(int argc, char **argv)
{
  auto log = LoggingService::Instance().getLogger("main");

  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
    ("help", "Show brief usage message")
    ("grabber", po::value<std::string>()->default_value("dummy"), "Cloud grabber type")
    ("imugrabber", po::value<std::string>(), "IMU grabber to use");
  // clang-format on

  /* Parse command line args */
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), args);
  }
  catch (po::error const &e)
  {
    log->critical("{}", e.what());
    return 1;
  }

  /* Show usage */
  if (args.count("help"))
  {
    std::cout << desc << "\n";
    return 1;
  }

  /* Create cloud grabber */
  typename ICloudGrabber<pcl::PointXYZRGBA>::Ptr grabber =
      CloudGrabberFactory<pcl::PointXYZRGBA>::Create(args["grabber"].as<std::string>());
  if (!grabber)
  {
    log->critical("Failed to create cloud grabber");
    return 1;
  }

  /* Create IMU grabber */
  IIMUGrabber_sptr imu = IMUGrabberFactory::Create(args["imugrabber"].as<std::string>());
  if (imu)
  {
    // TODO
    imu->setOrientation(Quaternion(Vector3(0.0f, 1.0f, 0.0f), -90.0f, DEGREES));
  }
  else
  {
    log->error("Failed to create IMU grabber");
  }

  Yukari::CloudGrabberTest::CloudGrabberVisualisation<pcl::PointXYZRGBA> viewer(grabber, imu);
  viewer.run();

  log->info("Exiting");
  return 0;
}
