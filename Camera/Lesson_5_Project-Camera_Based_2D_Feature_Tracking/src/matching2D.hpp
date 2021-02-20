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
  SIFT,
  SHITOMASI
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

double detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization = false);
double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &img,
                          std::string detectorType,
                          bool is_visualization = false);
double descKeypoints(std::vector<cv::KeyPoint> &keypoints,
                     cv::Mat &img,
                     cv::Mat &descriptors,
                     DescriptorType descriptor_type);
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                        std::vector<cv::KeyPoint> &kPtsRef,
                        cv::Mat &descSource,
                        cv::Mat &descRef,
                        std::vector<cv::DMatch> &matches,
                        MatcherType matcherType,
                        SelectorType selectorType);

#endif /* matching2D_hpp */
