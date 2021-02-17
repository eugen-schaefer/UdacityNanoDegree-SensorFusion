#include <numeric>
#include "matching2D.hpp"

using namespace std;

void visualize_detector_results(string window_name, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  cv::Mat visImage = img.clone();
  cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::namedWindow(window_name, 6);
  imshow(window_name, visImage);
  cv::waitKey(0);
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches,
                      MatcherType matcherType,
                      SelectorType selectorType) {
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  switch (matcherType) {
    case MatcherType::MAT_BF: {
      int normType = cv::NORM_L2;
      //cv::NORM_HAMMING does not work for AKAZE and SIFT;
      matcher = cv::BFMatcher::create(normType, crossCheck);
      break;
    }
    case MatcherType::MAT_FLANN: {
      matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      break;
    }
    default:break;
  }

  switch (selectorType) {
    case SelectorType::SEL_NN: {
      matcher->match(descSource, descRef, matches);
      // Finds the best match for each descriptor
      break;
    }
    case SelectorType::SEL_KNN: {
      std::vector<std::vector<cv::DMatch>> knn_matches;
      int k = 2;
      matcher->knnMatch(descSource, descRef, knn_matches, k);

      // TODO : filter matches using descriptor distance ratio test
      float dist_ratio_threshold{0.8f};
      for (auto &knn_match : knn_matches) {
        if (knn_match[0].distance / knn_match[1].distance < dist_ratio_threshold) {
          matches.push_back(knn_match[0]);
        }
      }
      break;
    }
    default:break;
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints,
                   cv::Mat &img,
                   cv::Mat &descriptors,
                   DescriptorType descriptor_type) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;

  std::string descriptor_type_name{};
  switch (descriptor_type) {
    case DescriptorType::AKAZE: {
      descriptor_type_name = "AKAZE";
      extractor = cv::AKAZE::create();
      break;
    }
    case DescriptorType::BRIEF: {
      descriptor_type_name = "BRIEF";
      extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
      break;
    }
    case DescriptorType::BRISK: {
      descriptor_type_name = "BRISK";
      int threshold{30};        // FAST/AGAST detection threshold score.
      int octaves{3};           // detection octaves (use 0 to do single scale)
      float patternScale{1.0f}; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
      extractor = cv::BRISK::create(threshold, octaves, patternScale);
      break;
    }
    case DescriptorType::FREAK: {
      descriptor_type_name = "FREAK";
      extractor = cv::xfeatures2d::FREAK::create();
      break;
    }
    case DescriptorType::ORB: {
      descriptor_type_name = "ORB";
      extractor = cv::ORB::create();
      break;
    }
    case DescriptorType::SIFT: {
      // see also https://stackoverflow.com/questions/27533203/how-do-i-use-sift-in-opencv-3-0-with-c
      descriptor_type_name = "SIFT";
      extractor = cv::SIFT::create();
      break;
    }
    default:break;
  }

  // perform feature description
  double t = (double) cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << descriptor_type_name << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  // compute detector parameters based on image size
  int blockSize =
      4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
  double maxOverlap = 0.0; // max. permissible overlap between two features in %
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

  double qualityLevel = 0.01; // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  double t = (double) cv::getTickCount();
  vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {

    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // visualize results
  if (is_visualization) {
    visualize_detector_results("Shi-Tomasi Corner Detector Results", keypoints, img);
  }

}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  // Detector parameters
  int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
  int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
  int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
  double k = 0.04;       // Harris parameter (see equation for details)

  cv::Mat dst = cv::Mat::zeros(img.size(), CV_32FC1);
  cv::cornerHarris(img, dst, blockSize, apertureSize, k);

  cv::Mat dst_norm, dst_norm_scaled;
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  // Look for prominent corners and instantiate keypoints
  double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
  for (int row_idx{0}; row_idx < dst_norm.rows; row_idx++) {
    for (int col_idx{0}; col_idx < dst_norm.cols; col_idx++) {
      int response = static_cast<int>(dst_norm.at<float>(row_idx, col_idx));
      if (response > minResponse) { // // only store points above a threshold
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f(row_idx, col_idx); //TODO(Eugen): check the order of col/row
        newKeyPoint.size = static_cast<float>(2 * apertureSize);
        newKeyPoint.response = static_cast<float>(response);

        // perform non-maximum suppression (NMS) in local neighbourhood around new key point
        bool is_overlap{};
        for (auto &keypoint : keypoints) {
          double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, keypoint);
          if (kptOverlap > maxOverlap) {
            if (newKeyPoint.response > keypoint.response) {
              keypoint = newKeyPoint;
              is_overlap = true;
              break;
            }
          }
        }
        // only add new key point if no overlap has been found in previous NMS
        if (!is_overlap) {
          keypoints.push_back(newKeyPoint);
        }
      }
    } // eof loop over cols
  } // eof loop over rows

  // visualize results    
  if (is_visualization) {
    visualize_detector_results("Harris Corner Detector Results", keypoints, img);
  }
}

void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  // difference between intensity of the central pixel and pixels of a circle around this pixel
  int threshold = 30;
  bool is_non_maxima_suppression{true};
  cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
  cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold, is_non_maxima_suppression, type);

  detector->detect(img, keypoints);

  // visualize results
  if (is_visualization) {
    visualize_detector_results("FAST Detector Results", keypoints, img);
  }
}

void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
  detector->detect(img, keypoints);

  // visualize results
  if (is_visualization) {
    visualize_detector_results("BRISK Detector Results", keypoints, img);
  }
}

void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(img, keypoints);

  // visualize results
  if (is_visualization) {
    visualize_detector_results("ORB Detector Results", keypoints, img);
  }
}

void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  cv::Mat descriptor;
  cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
  akaze->detectAndCompute(img, cv::noArray(), keypoints, descriptor);

  // visualize results
  if (is_visualization) {
    visualize_detector_results("AKAZE Detector Results", keypoints, img);
  }
}

void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool is_visualization) {
  cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
  detector->detect(img, keypoints);

  // visualize results
  if (is_visualization) {
    visualize_detector_results("SIFT Detector Results", keypoints, img);
  }
}
