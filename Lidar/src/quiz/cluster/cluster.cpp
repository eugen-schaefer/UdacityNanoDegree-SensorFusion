// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>> &points, KdTree *tree,
    float distanceTol) {
  // TODO: Fill out this function to return list of indices for each cluster

  std::vector<std::vector<int>> all_clusters;
  std::vector<int> single_cluster{};
  std::vector<bool> has_been_processed(points.size(), false);

  for (int i = 0; i < points.size(); ++i) {
    if (!has_been_processed.at(i)) {
      single_cluster.clear();
      Proximity(tree, points, i, has_been_processed, distanceTol,
                single_cluster);
      all_clusters.push_back(single_cluster);
    }
  }

  return all_clusters;
}

void Proximity(KdTree *kd_tree, const std::vector<std::vector<float>> &points,
               int id, std::vector<bool> &has_been_processed,
               float dist_tolerance, std::vector<int> &cluster) {
  has_been_processed.at(id) = true;
  cluster.push_back(id);

  std::vector<int> nearby_points =
      kd_tree->search(points.at(id), dist_tolerance);
  for (int point : nearby_points) {
    if (!has_been_processed.at(point)) {
      Proximity(kd_tree, points, point, has_been_processed, dist_tolerance,
                cluster);
    }
  }
}