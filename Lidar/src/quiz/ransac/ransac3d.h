#ifndef PLAYBACK_RANSAC3D_H
#define PLAYBACK_RANSAC3D_H

#include <pcl/visualization/pcl_visualizer.h>

#include <unordered_set>

std::unordered_set<int> RansacPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int maxIterations,
    float distanceTol);

#endif  // PLAYBACK_RANSAC3D_H
