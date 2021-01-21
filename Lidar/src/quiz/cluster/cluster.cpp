// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>> &points, KdTree *tree,
    float distanceTol, int min_number_points_in_cluster) {
  std::vector<std::vector<int>> all_clusters;
  std::vector<int> single_cluster{};
  std::vector<bool> has_been_processed(points.size(), false);

  for (int i = 0; i < points.size(); ++i) {
    if (!has_been_processed[i]) {
      single_cluster.clear();
      Proximity(tree, points, i, has_been_processed, distanceTol,
                single_cluster);
      if (single_cluster.size() >= min_number_points_in_cluster) {
        all_clusters.push_back(single_cluster);
      }
    }
  }

  return all_clusters;
}

void Proximity(KdTree *kd_tree, const std::vector<std::vector<float>> &points,
               int id, std::vector<bool> &has_been_processed,
               float dist_tolerance, std::vector<int> &cluster) {
  has_been_processed[id] = true;
  cluster.push_back(id);

  std::vector<int> nearby_points = kd_tree->search(points[id], dist_tolerance);
  for (int point : nearby_points) {
    if (!has_been_processed[point]) {
      Proximity(kd_tree, points, point, has_been_processed, dist_tolerance,
                cluster);
    }
  }
}