/** @file */

#include <boost/program_options.hpp>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/openni2_grabber.h>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCloudCapture/OpenNI2CloudGrabber.h>

#include "CloudGrabberVisualisation.h"

namespace po = boost::program_options;
using namespace Yukari::CloudCapture;

int main(int argc, char **argv)
{
  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
    ("help", "Show brief usage message")
    ("grabber", po::value<std::string>()->default_value("openni2"), "Cloud grabber type")
    ("device", po::value<std::string>()->default_value(""), "Device ID (for OpenNI2 grabber)")
    ("depthmode", po::value<int>()->default_value(0), "Depth capture mode (for OpenNI2 grabber)")
    ("imagemode", po::value<int>()->default_value(0), "Image capture mode (for OpenNI2 grabber)");
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

  /* Create cloud grabber */
  const std::string source = args["grabber"].as<std::string>();
  typename ICloudGrabber<pcl::PointXYZRGBA>::Ptr grabber;
  if (source == "openni2")
  {
    pcl::io::OpenNI2Grabber::Mode depthMode =
        pcl::io::OpenNI2Grabber::Mode(args["depthmode"].as<int>());
    pcl::io::OpenNI2Grabber::Mode imageMode =
        pcl::io::OpenNI2Grabber::Mode(args["imagemode"].as<int>());

    grabber = std::make_shared<OpenNI2CloudGrabber<pcl::PointXYZRGBA>>(
        args["device"].as<std::string>(), depthMode, imageMode);
  }

  if (!grabber)
    return 1;

  Yukari::CloudGrabberTest::CloudGrabberVisualisation<pcl::PointXYZRGBA> viewer(grabber);
  viewer.run();

  return 0;
}
