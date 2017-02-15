#include <chrono>
#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <YukariCommon/LoggingService.h>

int main(int argc, char **argv)
{
  auto logger = Yukari::Common::LoggingService::GetLogger("PCDViewer");

  if (argc != 2)
  {
    logger->error("Incorrect number of arguments!");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  logger->info("Loading point cloud: {}", argv[1]);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloud) == -1)
  {
    logger->error("Failed to load point cloid!");
    return 1;
  }

  logger->info("Starting viewer");
  pcl::visualization::CloudViewer viewer("PCD Viewer");
  viewer.showCloud(cloud);

  while (!viewer.wasStopped())
    std::this_thread::sleep_for(std::chrono::seconds(1));

  logger->info("Exiting");
  return 0;
}
