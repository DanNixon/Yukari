#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

int main(int argc, char *argv[])
{
  pcl::console::print_info("Loading cloud A (input)\n");
  CloudT::Ptr cloudA(new CloudT());
  pcl::io::loadPCDFile(argv[1], *cloudA);
  pcl::console::print_value("Num points in cloud A = %ld\n", cloudA->size());

  pcl::console::print_info("Loading cloud B (target)\n");
  CloudT::Ptr cloudB(new CloudT());
  pcl::io::loadPCDFile(argv[2], *cloudB);
  pcl::console::print_value("Num points in cloud B = %ld\n", cloudB->size());

  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);

  pcl::console::print_info("Filtering cloud A\n");
  CloudT::Ptr filteredCloudA(new CloudT());
  sor.setInputCloud(cloudA);
  sor.filter(*filteredCloudA);
  pcl::console::print_value("Num points in filtered cloud A = %ld\n", filteredCloudA->size());
  pcl::io::savePCDFileBinaryCompressed("out_filteredCloudA.pcd", *filteredCloudA);

  pcl::console::print_info("Filtering cloud B\n");
  CloudT::Ptr filteredCloudB(new CloudT());
  sor.setInputCloud(cloudB);
  sor.filter(*filteredCloudB);
  pcl::console::print_value("Num points in filtered cloud B = %ld\n", filteredCloudB->size());
  pcl::io::savePCDFileBinaryCompressed("out_filteredCloudB.pcd", *filteredCloudB);

  pcl::VoxelGrid<PointT> vox;
  const float leaf = 0.01f;
  vox.setLeafSize(leaf, leaf, leaf);

  pcl::console::print_info("Downsampling cloud A\n");
  CloudT::Ptr downsampledCloudA(new CloudT());
  vox.setInputCloud(filteredCloudA);
  vox.filter(*downsampledCloudA);
  pcl::console::print_value("Num points in downsampled cloud A = %ld\n", downsampledCloudA->size());
  pcl::io::savePCDFileBinaryCompressed("out_downsampledCloudA.pcd", *downsampledCloudA);

  pcl::console::print_info("Downsampling cloud B\n");
  CloudT::Ptr downsampledCloudB(new CloudT());
  vox.setInputCloud(filteredCloudB);
  vox.filter(*downsampledCloudB);
  pcl::console::print_value("Num points in downsampled cloud B = %ld\n", downsampledCloudB->size());
  pcl::io::savePCDFileBinaryCompressed("out_downsampledCloudB.pcd", *downsampledCloudB);

  pcl::NormalEstimationOMP<PointT, NormalT> ne;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);

  pcl::console::print_info("Normal estimation cloud A\n");
  ne.setInputCloud(downsampledCloudA);
  NormalCloudT::Ptr normalsA(new NormalCloudT);
  ne.compute(*normalsA);
  pcl::io::savePCDFileBinaryCompressed("out_normalsA.pcd", *normalsA);

  for (int i = 0; i < normalsA->points.size(); i++)
  {
    if (!pcl::isFinite<pcl::Normal>(normalsA->points[i]))
      PCL_WARN("normalsA[%d] is not finite\n", i);
  }

  pcl::console::print_info("Normal extimation cloud B\n");
  ne.setInputCloud(downsampledCloudB);
  NormalCloudT::Ptr normalsB(new NormalCloudT);
  ne.compute(*normalsB);
  pcl::io::savePCDFileBinaryCompressed("out_normalsB.pcd", *normalsB);

  for (int i = 0; i < normalsB->points.size(); i++)
  {
    if (!pcl::isFinite<pcl::Normal>(normalsB->points[i]))
      PCL_WARN("normalsB[%d] is not finite\n", i);
  }

  pcl::FPFHEstimationOMP<PointT, NormalT, FeatureT> pfh;
  pfh.setSearchMethod(tree);
  pfh.setRadiusSearch(0.05);

  pcl::console::print_info("PFH feature finding cloud A\n");
  FeatureCloudT::Ptr pfhsA(new FeatureCloudT);
  pfh.setInputCloud(downsampledCloudA);
  pfh.setInputNormals(normalsA);
  pfh.compute(*pfhsA);
  pcl::console::print_value("Num points in feature cloud A = %ld\n", pfhsA->size());
  pcl::io::savePCDFileBinaryCompressed("out_pfhsA.pcd", *pfhsA);

  pcl::console::print_info("PFH feature finding cloud B\n");
  FeatureCloudT::Ptr pfhsB(new FeatureCloudT);
  pfh.setInputCloud(downsampledCloudB);
  pfh.setInputNormals(normalsB);
  pfh.compute(*pfhsB);
  pcl::console::print_value("Num points in feature cloud B = %ld\n", pfhsB->size());
  pcl::io::savePCDFileBinaryCompressed("out_pfhsB.pcd", *pfhsB);

  // PAIRWISE OPERATIONS START HERE

  pcl::console::print_info("Correspondence estimation\n");
  pcl::registration::CorrespondenceEstimation<FeatureT, FeatureT> ce;
  ce.setInputTarget(pfhsA);
  ce.setInputCloud(pfhsB);
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  ce.determineCorrespondences(*correspondences);
  pcl::console::print_value("Num correspondences = %ld\n", correspondences->size());

  pcl::console::print_info("Save correspondences\n");
  {
    std::ofstream file;
    file.open("out_correspondences.txt");
    for (size_t i = 0; i < correspondences->size(); ++i)
      file << correspondences->at(i) << '\n';
    file.close();
  }

  pcl::console::print_info("Correspondence rejection\n");
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
  sac.setMaxIterations(1000);
  sac.setInlierThreshold(0.2);
  sac.setTargetCloud(downsampledCloudA);
  sac.setInputCloud(downsampledCloudB);
  sac.setInputCorrespondences(correspondences);
  pcl::CorrespondencesPtr keptCorrespondences(new pcl::Correspondences());
  sac.getCorrespondences(*keptCorrespondences);
  pcl::console::print_value("Num correspondences after rejection = %ld\n",
                            keptCorrespondences->size());

  pcl::console::print_info("Save kept correspondences\n");
  {
    std::ofstream file;
    file.open("out_keptCorrespondences.txt");
    for (size_t i = 0; i < keptCorrespondences->size(); ++i)
      file << keptCorrespondences->at(i) << '\n';
    file.close();
  }

  pcl::console::print_info("Transformation estimation\n");
  Eigen::Matrix4f transform;
  pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
  svd.estimateRigidTransformation(*downsampledCloudB, *downsampledCloudA, *keptCorrespondences,
                                  transform);

  pcl::console::print_info("Save initial estimate\n");
  {
    std::ofstream file;
    file.open("out_transformationInitial.txt");
    file << transform << '\n';
    file.close();
  }
  CloudT::Ptr transformedInitialB(new CloudT);
  pcl::transformPointCloud(*filteredCloudB, *transformedInitialB, transform);
  pcl::io::savePCDFileBinaryCompressed("out_transformedInitialB.pcd", *transformedInitialB);

  pcl::console::print_info("Trim clouds to correspondences\n");
  CloudT::Ptr trimmedA(new CloudT);
  CloudT::Ptr trimmedB(new CloudT);
  size_t idxC;
  size_t checkLim = std::min(downsampledCloudA->size(), downsampledCloudB->size());
  for (size_t i = 0; i < keptCorrespondences->size(); ++i)
  {
    idxC = keptCorrespondences->at(i).index_query;
    if (idxC < checkLim)
    {
      PointT &ptA = downsampledCloudA->at(idxC);
      PointT &ptB = downsampledCloudB->at(idxC);
      trimmedA->push_back(ptA);
      trimmedB->push_back(ptB);
    }
    else
    {
      PCL_WARN("Point index error: %d\n", idxC);
    }
  }
  pcl::console::print_value("Points in trimmed cloud A = %ld\n", trimmedA->size());
  pcl::console::print_value("Points in trimmed cloud B = %ld\n", trimmedB->size());
  pcl::io::savePCDFileBinaryCompressed("out_trimmedA.pcd", *trimmedA);
  pcl::io::savePCDFileBinaryCompressed("out_trimmedB.pcd", *trimmedB);

  pcl::console::print_info("Fine alignment\n");
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(0.5);
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(0.01);
  icp.setInputTarget(trimmedA);
  icp.setInputCloud(trimmedB);
  CloudT::Ptr registeredB(new CloudT);
  icp.align(*registeredB, transform);
  pcl::console::print_value("ICP has converged: %d\n", icp.hasConverged());
  pcl::console::print_value("ICP fitness score = %f\n", icp.getFitnessScore());
  transform = icp.getFinalTransformation();

  pcl::console::print_info("Save final registration\n");
  {
    std::ofstream file;
    file.open("out_transformationFinal.txt");
    file << transform << '\n';
    file.close();
  }
  CloudT::Ptr transformedFinalB(new CloudT);
  pcl::transformPointCloud(*filteredCloudB, *transformedFinalB, transform);
  pcl::io::savePCDFileBinaryCompressed("out_transformedFinalB.pcd", *transformedFinalB);

  pcl::console::print_info("Done\n");
  {
    std::string str;
    std::getline(std::cin, str);
  }

  return 0;
}