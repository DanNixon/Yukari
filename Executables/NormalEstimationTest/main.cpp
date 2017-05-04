#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int arc, char **argv)
{
  // load point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile(argv[1], *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredInputCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> voxelFilter;
  voxelFilter.setLeafSize(0.01, 0.01, 0.01);
  voxelFilter.setInputCloud(cloud);
  voxelFilter.filter(*filteredInputCloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(filteredInputCloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other
  // search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.1);

  // Compute the features
  ne.compute(*cloud_normals);

  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(filteredInputCloud, cloud_normals);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
  return 0;
}