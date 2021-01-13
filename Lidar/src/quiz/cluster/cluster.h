#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

#include "kdtree.h"

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>> &points, KdTree *tree,
    float distanceTol);

void Proximity(KdTree *kd_tree, const std::vector<std::vector<float>> &points,
               int id, std::vector<bool> &has_been_processed,
               float dist_tolerance, std::vector<int> &cluster);

#endif  // PLAYBACK_CLUSTER_H
