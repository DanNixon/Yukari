// http://pointclouds.org/documentation/tutorials/normal_distributions_transform.php

#include <iostream>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <YukariCommon/LoggingService.h>
#include <YukariProcessing/CloudOperations.h>

using namespace Yukari::Common;
using namespace Yukari::Processing;

int main(int argc, char **argv)
{
  auto logger = LoggingService::Instance().getLogger("main");

  /* Load target cloud */
  std::string targetCloudFilename(argv[1]);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(targetCloudFilename, *targetCloud) == -1)
  {
    logger->critical("Cannot read target cloud file \"{}\"", targetCloudFilename);
    return 1;
  }
  targetCloud = CloudOperations<pcl::PointXYZRGBA>::RemoveNaNFromCloud(targetCloud);
  logger->info("Loaded target cloud \"{}\" of {} points", targetCloudFilename, targetCloud->size());

  /* Load input cloud */
  std::string inputCloudFilename(argv[2]);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(inputCloudFilename, *inputCloud) == -1)
  {
    logger->critical("Cannot read input cloud file \"{}\"", inputCloudFilename);
    return 1;
  }
  inputCloud = CloudOperations<pcl::PointXYZRGBA>::RemoveNaNFromCloud(inputCloud);
  logger->info("Loaded input cloud \"{}\" of {} points", inputCloudFilename, inputCloud->size());

  /* FIlter (downsample) input cloud */
  auto filteredInputCloud = CloudOperations<pcl::PointXYZRGBA>::DownsampleVoxelFilter(inputCloud);

  pcl::NormalDistributionsTransform<pcl::PointXYZRGBA, pcl::PointXYZRGBA> ndt;

  ndt.setTransformationEpsilon(0.005);
  ndt.setStepSize(0.01);
  ndt.setResolution(0.1);

  ndt.setMaximumIterations(35);

  ndt.setInputSource(filteredInputCloud);
  ndt.setInputTarget(targetCloud);

  /* Run alignment (operating on transformed point cloud so no/identity initial guess) */
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedInputCloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  ndt.align(*transformedInputCloud, Eigen::Matrix4f::Identity());

  if (ndt.hasConverged())
    logger->info("Convergence reached");
  else
    logger->warn("Convergence not reached");
  logger->info("Normal Distributions Transform score: {}", ndt.getFitnessScore());

  /* Transform the original input cloud */
  pcl::transformPointCloud(*inputCloud, *transformedInputCloud, ndt.getFinalTransformation());

  pcl::io::savePCDFileASCII("transformed.pcd", *transformedInputCloud);

  return 0;
}