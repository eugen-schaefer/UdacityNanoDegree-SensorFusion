#include "ransac3d.h"

#include <random>

std::unordered_set<int> RansacPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int maxIterations,
    float distanceTol) {
  std::unordered_set<int> inliersResult;
  std::unordered_set<int> temp_inliers_result;
  srand(time(NULL));

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

    // Fit plane
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
      // Measure distance between every point and fitted plane
      float d{fabs(A * point.x + B * point.y + C * point.z + D) /
              std::sqrt(A * A + B * B + C * C)};
      // If distance is smaller than threshold count it as inlier
      if (d < distanceTol) {
        temp_inliers_result.insert(j);
      }
    }

    if (temp_inliers_result.size() > inliersResult.size()) {
      inliersResult = temp_inliers_result;
    }
  }
  // Return indicies of inliers from fitted plane with most inliers
  return inliersResult;
}