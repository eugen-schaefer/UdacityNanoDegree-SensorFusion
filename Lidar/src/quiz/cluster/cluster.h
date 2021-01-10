#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

#include <chrono>
#include <string>

#include "../../render/box.h"
#include "../../render/render.h"
#include "kdtree.h"

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(
    std::vector<std::vector<float>> points);
void render2DTree(Node *node, pcl::visualization::PCLVisualizer::Ptr &viewer,
                  Box window, int &iteration, uint depth=0);
std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>> &points, KdTree *tree,
    float distanceTol);

//pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);

#endif  // PLAYBACK_CLUSTER_H
