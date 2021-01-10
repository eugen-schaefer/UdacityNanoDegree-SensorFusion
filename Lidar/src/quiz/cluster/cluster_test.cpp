#include "kdtree.h"
#include "cluster.h"
#include "gtest/gtest.h"

struct KDTreeTest : public ::testing::Test {
  KdTree unit{};
  std::vector<std::vector<float>> points = {
      {-6.2, 7},   {-6.3, 8.4},  {-5.2, 7.1}, {-5.7, 6.3},
      {7.2, 6.1},  {8.0, 5.3},   {7.2, 7.1},  {0.2, -7.1},
      {1.7, -6.9}, {-1.2, -7.2}, {2.2, -8.9}};
};

TEST_F(KDTreeTest, Proxymity) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }
  float dist_tolerance{3.f};
  std::vector<bool> has_been_processed(points.size(), false);
  std::vector<int> actual_value{};
  std::vector<int> expected_value{7, 8, 9, 10};

  Proximity(&unit, points, 7, has_been_processed, dist_tolerance,
            actual_value);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTreeTest, EuclideanClusters) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }

  std::vector<std::vector<int>> expected_values{
      {0, 1, 2, 3}, {4, 5, 6}, {7, 8, 9, 10}};
  // Ensure that values in every expected cluster are sorted in ascending order
  for (auto &cluster : expected_values) {
    std::sort(cluster.begin(), cluster.end());
  }
  // Ensure that all expected cluster are sorted in ascending order based on the first
  // entry in cluster
  sort(expected_values.begin(), expected_values.end(),
       [](std::vector<int> a, std::vector<int> b) {
         return (a.at(0) < b.at(0));
       });

  std::vector<std::vector<int>> actual_values =
      euclideanCluster(points, &unit, 3.f);
  EXPECT_EQ(expected_values.size(), actual_values.size());

  // Ensure that values in every calculated cluster are sorted in ascending order
  for (auto &cluster : actual_values) {
    std::sort(cluster.begin(), cluster.end());
  }
  // Ensure that all calculated cluster are sorted in ascending order based on the first
  // entry in cluster
  sort(actual_values.begin(), actual_values.end(),
       [](std::vector<int> a, std::vector<int> b) {
         return (a.at(0) < b.at(0));
       });

  EXPECT_EQ(expected_values, actual_values);
}