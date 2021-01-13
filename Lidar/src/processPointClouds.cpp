// Functions for processing point clouds, partially from PCL lib and partially self-written

#ifndef PROCESS_POINT_COUDS_H_
#define PROCESS_POINT_COUDS_H_

#include "processPointClouds.h"

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                              float filterRes,
                                                                              Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

  // First, crop the image
  typename pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloud);
  region.filter(*cropped_cloud);

  // Then, eliminate points reflected on the roof
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.f, -1.f));
  roof.setMax(Eigen::Vector4f(2.6f, 1.7f, 0.4f, 1.f));
  roof.setInputCloud(cropped_cloud);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point :indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cropped_cloud);

  // Finally, downsample the cropped image
  typename pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>);
  typename pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cropped_cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*downsampled_cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return downsampled_cloud;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                                                                            typename pcl::PointCloud<
                                                                                                PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacles_cloud(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>);


  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  // Extract the points in cloud referenced by inliers as a separate point cloud:
  extract.setNegative(false);
  extract.filter(*road_cloud);

  // Retrieve indices to all points in cloud except those referenced by inliers:
  extract.setNegative(true);
  extract.filter(*obstacles_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
      segmentation_result(obstacles_cloud, road_cloud);
  return segmentation_result;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PCLSegmentPlane(typename pcl::PointCloud<
    PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in this function to find inliers for the cloud.
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty()) {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<
    PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> extracted_clusters;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance); // in meter
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(extracted_clusters);

  for (const auto &cluster : extracted_clusters) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (int index : cluster.indices) {
      cloud_cluster->push_back((*cloud)[index]);
    };
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box{};
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path>
      paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;

}

#endif // PROCESS_POINT_COUDS_H_