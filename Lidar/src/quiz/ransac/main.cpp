#include <string>

#include "ransac2d.h"
#include "ransac3d.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include "../../render/render.h"

pcl::visualization::PCLVisualizer::Ptr initScene(const std::string &name) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer(name));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData2D() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

int main() {
  // ************** Toggle here between 2D and 3D clustering *******************
  bool is_rendering_2D{};  // true: 2D rendering, false: 3D rendering

  pcl::visualization::PCLVisualizer::Ptr viewer;
  std::unordered_set<int> inliers{};
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{};
  if (is_rendering_2D) {
    // Create viewer
    viewer = initScene("2D Viewer");

    // Create data
    cloud = CreateData2D();

    // TODO: Change the max iteration and distance tolerance arguments for
    // Ransac function
    inliers = Ransac(cloud, 50, 1.0);
  } else {
    // Create viewer
    viewer = initScene("3D Viewer");

    // Create data
    cloud = CreateData3D();

    // TODO: Change the max iteration and distance tolerance arguments for
    // RansacPlane function
    auto startTime = std::chrono::steady_clock::now();
    inliers = RansacPlane<pcl::PointXYZ>(cloud, 50, 0.2f);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds"
              << std::endl;

    // This code is only for timing comparison reasons
    ProcessPointClouds<pcl::PointXYZ> point_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
              pcl::PointCloud<pcl::PointXYZ>::Ptr>
        segmentCloud = point_processor.PCLSegmentPlane(cloud, 500000000, 1.2f);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render point cloud with inliers and outliers
  if (!inliers.empty()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}