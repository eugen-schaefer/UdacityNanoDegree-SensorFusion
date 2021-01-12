#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> RansacPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                    int maxIterations,
                                    float distanceTol) {
  std::unordered_set<int> inliersResult;
  std::unordered_set<int> temp_inliers_result;
  srand(time(NULL));

  // TODO: Fill in this function
  // For max iterations
  for (size_t i = 0; i < maxIterations; ++i) {
    // Randomly sample subset
    std::random_device rnd_dev;
    std::mt19937 mt_engine(rnd_dev());
    std::uniform_int_distribution<> generate_number(0, cloud->points.size());
    int index1, index2, index3;
    do {
      index1 = generate_number(mt_engine);
      index2 = generate_number(mt_engine);
      index3 = generate_number(mt_engine);
    } while ((index1 == index2) || (index1 == index3) || (index2 == index3));

    // Fit line
    float x1{cloud->points[index1].x};
    float y1{cloud->points[index1].y};
    float z1{cloud->points[index1].z};
    float x2{cloud->points[index2].x};
    float y2{cloud->points[index2].y};
    float z2{cloud->points[index2].z};
    float x3{cloud->points[index3].x};
    float y3{cloud->points[index3].y};
    float z3{cloud->points[index3].z};

    float A{(y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1)};
    float B{(z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1)};
    float C{(x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)};
    float D{-(A * x1 + B * y1 + C * z1)};

    temp_inliers_result.clear();
    for (int j = 0; j < cloud->size(); ++j) {
      auto point = cloud->points.at(j);
      // Measure distance between every point and fitted line
      float d{fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C)};
      // If distance is smaller than threshold count it as inlier
      if (d < distanceTol) {
        temp_inliers_result.insert(j);
      }
    }

    if (temp_inliers_result.size() > inliersResult.size()) {
      inliersResult = temp_inliers_result;
    }

  }
  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;
}

int main() {
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // TODO: Change the max iteration and distance tolerance arguments for RansacPlane function
  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2f);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "RANSAC plane took " << elapsedTime.count() << " milliseconds" << std::endl;

  // This code is only for timing comparison reasons
  ProcessPointClouds<pcl::PointXYZ> point_processor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentCloud = point_processor.SegmentPlane(cloud, 500000000, 1.2f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 3D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }

}
