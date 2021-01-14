#ifndef PLAYBACK_RANSAC2D_H
#define PLAYBACK_RANSAC2D_H

#include <pcl/visualization/pcl_visualizer.h>

#include <unordered_set>

std::unordered_set<int> Ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               int maxIterations, float distanceTol);

#endif  // PLAYBACK_RANSAC2D_H
