#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

#include <chrono>
#include <string>
#include <utility>

#include "../../render/box.h"
#include "../../render/render.h"
#include "kdtree.h"

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(
    std::vector<std::vector<float>> points);
void render2DTree(Node *node, pcl::visualization::PCLVisualizer::Ptr &viewer,
                  Box window, int &iteration, uint depth = 0);
std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>> &points, KdTree *tree,
    float distanceTol);

void Proximity(KdTree *kd_tree, const std::vector<std::vector<float>> &points, int id, std::vector<bool> &has_been_processed, float dist_tolerance, std::vector<int>& cluster);

#endif  // PLAYBACK_CLUSTER_H
