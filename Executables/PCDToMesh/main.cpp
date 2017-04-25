/** @file */

#include <iostream>

#include <boost/program_options.hpp>
#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include <pcl/io/file_io.h>
#else
#include <pcl/io/auto_io.h>
#endif

#include <YukariCommon/LoggingService.h>
#include <YukariProcessing/GenerateMeshFromPointCloud.h>

using namespace Yukari::Common;
using namespace Yukari::Processing;
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
    ("loglevel", po::value<std::string>()->default_value("trace"), "Global log level")
    ("cloud", po::value<std::string>(), "Point cloud file (.pcd)")
    ("mesh", po::value<std::string>(), "Output mesh file");
  // clang-format on

  /* Parse command line args */
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), args);
  }
  catch (po::error const &e)
  {
    logger->critical("{}", e.what());
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

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  /* Load point cloud */
  std::string cloudFilename(args["cloud"].as<std::string>());
  logger->info("Loading point cloud: {}", cloudFilename);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(cloudFilename, *cloud) == -1)
  {
    logger->error("Failed to load point cloid!");
    return 1;
  }

  /* Get parameters */
  GenerateMeshFromPointCloud<pcl::PointXYZRGBA>::Parameters params;
  // TODO

  /* Mesh estimation */
  GenerateMeshFromPointCloud<pcl::PointXYZRGBA> meshEstimator;
  pcl::PolygonMesh::Ptr mesh = meshEstimator.estimateSingle(cloud, params);

  /* Save generated mesh */
  std::string meshFilename(args["mesh"].as<std::string>());
  logger->info("Saving mesh: {}", meshFilename);
  pcl::io::save(meshFilename, *mesh);

  logger->info("Exiting");
  return 0;
}
