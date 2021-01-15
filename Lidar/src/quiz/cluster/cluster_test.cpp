#include "cluster.h"

#include <algorithm>

#include "gtest/gtest.h"
#include "kdtree.h"

struct Cluster2DTest : public ::testing::Test {
  KdTree unit{};
  std::vector<std::vector<float>> points = {
      {-6.2f, 7.0f}, {-6.3f, 8.4f},  {-5.2f, 7.1f}, {-5.7f, 6.3f},
      {7.2f, 6.1f},  {8.0f, 5.3f},   {7.2f, 7.1f},  {0.2f, -7.1f},
      {1.7f, -6.9f}, {-1.2f, -7.2f}, {2.2f, -8.9f}};
};

struct Cluster3DTest : public ::testing::Test {
  KdTree unit{};
  std::vector<std::vector<float>> points_3d = {
      {-6.2f, 7.0f, 3.1f},   {-6.3f, 8.4f, 2.8f},  {-5.2f, 7.1f, 4.0f},
      {-5.7f, 6.3f, 3.4f},   {7.2f, 6.1f, -25.8f}, {8.0f, 5.3f, -28.1f},
      {7.2f, 7.1f, -27.3f},  {0.2f, -7.1f, 45.6f}, {1.7f, -6.9f, 43.0f},
      {-1.2f, -7.2f, 44.1f}, {2.2f, -8.9f, 44.8f}};
};

TEST_F(Cluster2DTest, Proxymity) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }
  float dist_tolerance{3.f};
  std::vector<bool> has_been_processed(points.size(), false);
  std::vector<int> actual_value{};
  std::vector<int> expected_value{7, 8, 9, 10};

  Proximity(&unit, points, 7, has_been_processed, dist_tolerance, actual_value);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(Cluster2DTest, EuclideanClusters) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }

  std::vector<std::vector<int>> expected_values{
      {0, 1, 2, 3}, {4, 5, 6}, {7, 8, 9, 10}};
  // Ensure that values in every expected cluster are sorted in ascending order
  for (auto &cluster : expected_values) {
    std::sort(cluster.begin(), cluster.end());
  }
  // Ensure that all expected cluster are sorted in ascending order based on the
  // first entry in cluster
  sort(expected_values.begin(), expected_values.end(),
       [](std::vector<int> a, std::vector<int> b) {
         return (a.at(0) < b.at(0));
       });

  std::vector<std::vector<int>> actual_values =
      euclideanCluster(points, &unit, 3.f);
  EXPECT_EQ(expected_values.size(), actual_values.size());

  // Ensure that values in every calculated cluster are sorted in ascending
  // order
  for (auto &cluster : actual_values) {
    std::sort(cluster.begin(), cluster.end());
  }
  // Ensure that all calculated cluster are sorted in ascending order based on
  // the first entry in cluster
  sort(actual_values.begin(), actual_values.end(),
       [](std::vector<int> a, std::vector<int> b) {
         return (a.at(0) < b.at(0));
       });

  EXPECT_EQ(expected_values, actual_values);
}

TEST_F(Cluster3DTest, Proxymity) {
  for (int i = 0; i < points_3d.size(); i++) {
    unit.insert(points_3d[i], i);
  }
  float dist_tolerance{3.f};
  std::vector<bool> has_been_processed(points_3d.size(), false);
  std::vector<int> actual_value{};
  std::vector<int> expected_value{7, 8, 9, 10};

  Proximity(&unit, points_3d, 7, has_been_processed, dist_tolerance,
            actual_value);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(Cluster3DTest, EuclideanClusters) {
  for (int i = 0; i < points_3d.size(); i++) {
    unit.insert(points_3d[i], i);
  }

  std::vector<std::vector<int>> expected_values{
      {0, 1, 2, 3}, {4, 5, 6}, {7, 8, 9, 10}};
  // Ensure that values in every expected cluster are sorted in ascending order
  for (auto &cluster : expected_values) {
    std::sort(cluster.begin(), cluster.end());
  }
  // Ensure that all expected cluster are sorted in ascending order based on the
  // first entry in cluster
  sort(expected_values.begin(), expected_values.end(),
       [](std::vector<int> a, std::vector<int> b) {
         return (a.at(0) < b.at(0));
       });

  std::vector<std::vector<int>> actual_values =
      euclideanCluster(points_3d, &unit, 3.f);
  EXPECT_EQ(expected_values.size(), actual_values.size());

  // Ensure that values in every calculated cluster are sorted in ascending
  // order
  for (auto &cluster : actual_values) {
    std::sort(cluster.begin(), cluster.end());
  }
  // Ensure that all calculated cluster are sorted in ascending order based on
  // the first entry in cluster
  sort(actual_values.begin(), actual_values.end(),
       [](std::vector<int> a, std::vector<int> b) {
         return (a.at(0) < b.at(0));
       });

  EXPECT_EQ(expected_values, actual_values);
}