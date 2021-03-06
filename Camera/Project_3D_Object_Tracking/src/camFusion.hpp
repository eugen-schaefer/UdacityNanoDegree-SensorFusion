
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>

#include <opencv2/core.hpp>
#include <vector>

#include "dataStructures.h"

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes,
                         std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor, cv::Mat &P_rect_xx,
                         cv::Mat &R_rect_xx, cv::Mat &RT);
void clusterKptMatchesWithROI(BoundingBox &bounding_box,
                              std::vector<cv::KeyPoint> &kpts_prev,
                              std::vector<cv::KeyPoint> &kpts_curr,
                              std::vector<cv::DMatch> &kpt_matches);
void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bb_best_matches, DataFrame &prev_frame,
                        DataFrame &curr_frame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize,
                   cv::Size imageSize, bool bWait = true);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate,
                      double &TTC, cv::Mat *visImg = nullptr);
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate,
                     double &TTC);
#endif /* camFusion_hpp */
