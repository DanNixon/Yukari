#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <YukariCommon/LoggingService.h>

int main(int argc, char **argv)
{
  auto logger = Yukari::Common::LoggingService::GetLogger("PCDToMesh");

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

  /* TODO */

  logger->info("Exiting");
  return 0;
}
