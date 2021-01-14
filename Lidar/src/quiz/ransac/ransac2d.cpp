#include "ransac2d.h"

#include <random>

std::unordered_set<int> Ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               int maxIterations, float distanceTol) {
  std::unordered_set<int> inliers_result;
  std::unordered_set<int> temp_inliers_result;
  srand(time(NULL));

  // For max iterations
  for (size_t i = 0; i < maxIterations; ++i) {
    // Randomly sample subset
    std::random_device rnd_dev;
    std::mt19937 mt_engine(rnd_dev());
    std::uniform_int_distribution<> generate_number(0, cloud->points.size());
    int index1, index2;
    do {
      index1 = generate_number(mt_engine);
      index2 = generate_number(mt_engine);
    } while (index1 == index2);

    // Fit line
    float x1{cloud->points[index1].x};
    float y1{cloud->points[index1].y};
    float x2{cloud->points[index2].x};
    float y2{cloud->points[index2].y};

    float A{y1 - y2};
    float B{x2 - x1};
    float C{x1 * y2 - x2 * y1};

    temp_inliers_result.clear();
    for (int j = 0; j < cloud->size(); ++j) {
      auto point = cloud->points.at(j);
      // Measure distance between every point and fitted line
      float d{fabs(A * point.x + B * point.y + C) / std::sqrt(A * A + B * B)};
      // If distance is smaller than threshold count it as inlier
      if (d < distanceTol) {
        temp_inliers_result.insert(j);
      }
    }

    if (temp_inliers_result.size() > inliers_result.size()) {
      inliers_result = temp_inliers_result;
    }
  }
  // Return indicies of inliers from fitted line with most inliers
  return inliers_result;
}