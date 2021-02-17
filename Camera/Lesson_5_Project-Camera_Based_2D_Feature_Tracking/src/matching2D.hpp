#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"

enum class DetectorType {
  AKAZE,
  BRISK,
  FAST,
  HARRIS,
  ORB,
  SHITOMASI,
  SIFT
};

enum class DescriptorType {
  AKAZE,
  BRIEF,
  BRISK,
  FREAK,
  ORB,
  SIFT
};

enum class MatcherType {
  MAT_BF,
  MAT_FLANN
};

enum class SelectorType {
  SEL_NN, // nearest neighbor (best match)
  SEL_KNN // k nearest neighbors (k=2)
};

void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints,
                        cv::Mat &img,
                        std::string detectorType,
                        bool is_visualization = false);
void descKeypoints(std::vector<cv::KeyPoint> &keypoints,
                   cv::Mat &img,
                   cv::Mat &descriptors,
                   DescriptorType descriptor_type);
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches,
                      MatcherType matcherType,
                      SelectorType selectorType);

#endif /* matching2D_hpp */
