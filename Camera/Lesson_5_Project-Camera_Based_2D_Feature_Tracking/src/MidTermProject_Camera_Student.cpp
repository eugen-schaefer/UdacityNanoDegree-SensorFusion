/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <iomanip>
#include <deque>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {

  /* INIT VARIABLES AND DATA STRUCTURES */

  // detector / descriptor combinations
  DetectorType detector_type{DetectorType::ORB};
  DescriptorType descriptor_type{DescriptorType::BRISK}; // BRIEF, ORB, FREAK, AKAZE, SIFT
  MatcherType matcher_type{MatcherType::MAT_BF};        // MAT_BF, MAT_FLANN
  SelectorType selector_type{SelectorType::SEL_KNN};       // SEL_NN, SEL_KNN

  bool is_focus_on_leading_vehicle{true};


  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
  string imgFileType = ".png";
  int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
  int imgEndIndex = 9;   // last file index to load
  int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  // misc
  int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
  std::deque<DataFrame> dataBuffer{}; // list of data frames which are held in memory at the same time
  bool is_visualization_on{true};            // visualize results

  struct KeypointStatisticsType {
    int number_keypoints_on_preceding_vehicle{};
    float size_mean{};
    float size_standard_deviation{};
    int number_keypoint_matches{};
    float keypoint_detection_time_ms{};
    float descriptor_extraction_time_ms{};
    float keypoint_matching_time_ms{};
  };

  std::vector<KeypointStatisticsType> keypoints_statistics;

  /* MAIN LOOP OVER ALL IMAGES */

  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
    /* LOAD IMAGE INTO BUFFER */

    // assemble filenames for current index
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    // load image from file and convert to grayscale
    cv::Mat img, imgGray;
    img = cv::imread(imgFullFilename);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    //// STUDENT ASSIGNMENT
    //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

    // push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = imgGray;
    if (dataBuffer.size() > dataBufferSize - 1) {
      dataBuffer.pop_front();
    }
    dataBuffer.push_back(frame);

    //// EOF STUDENT ASSIGNMENT
    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

    /* DETECT IMAGE KEYPOINTS */

    // extract 2D keypoints from current image
    vector<cv::KeyPoint> keypoints; // create empty feature list for current image

    double detector_execution_time_ms{};

    //// STUDENT ASSIGNMENT
    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    switch (detector_type) {
      case DetectorType::SHITOMASI: {
        detector_execution_time_ms = detKeypointsShiTomasi(keypoints, imgGray, false);
        break;
      }
      case DetectorType::HARRIS: {
        detector_execution_time_ms = detKeypointsHarris(keypoints, imgGray, false);
        break;
      }
      case DetectorType::FAST: {
        detector_execution_time_ms = detKeypointsFAST(keypoints, imgGray, false);
        break;
      }
      case DetectorType::BRISK: {
        detector_execution_time_ms = detKeypointsBRISK(keypoints, imgGray, false);
        break;
      }
      case DetectorType::ORB: {
        detector_execution_time_ms = detKeypointsORB(keypoints, imgGray, false);
        break;
      }
      case DetectorType::AKAZE: {
        detector_execution_time_ms = detKeypointsAKAZE(keypoints, imgGray, false);
        break;
      }
      case DetectorType::SIFT: {
        detector_execution_time_ms = detKeypointsSIFT(keypoints, imgGray, false);
        break;
      }
      default:
        break;
    }
    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TASK MP.3 -> only keep keypoints on the preceding vehicle
    float topleft_corner_x = 535.f, topleft_corner_y = 180.f, width = 180.f, height = 150.f;
    cv::Rect bounding_box(topleft_corner_x, topleft_corner_y, width, height);
    if (is_focus_on_leading_vehicle) {
      for (auto it = keypoints.begin(); it != keypoints.end();) {
        if (!bounding_box.contains(it->pt)) {
          it = keypoints.erase(it);
        } else {
          ++it;
        }
      }
    }
    //// EOF STUDENT ASSIGNMENT

    // optional : limit number of keypoints (helpful for debugging and learning)
    bool bLimitKpts = false;
    if (bLimitKpts) {
      int maxKeypoints = 50;
      // there is no response info, so keep the first 50 as they are sorted in descending quality order
      if (detector_type == DetectorType::SIFT) {
        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
      }
      cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
      cout << " NOTE: Keypoints have been limited!" << endl;
    }

    // push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;
    cout << "#2 : DETECT KEYPOINTS done" << endl;

    /* EXTRACT KEYPOINT DESCRIPTORS */

    //// STUDENT ASSIGNMENT
    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
    //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
    double descriptor_extraction_time_ms{};
    cv::Mat descriptors;
    descriptor_extraction_time_ms = descKeypoints((dataBuffer.end() - 1)->keypoints,
                                                  (dataBuffer.end() - 1)->cameraImg,
                                                  descriptors,
                                                  descriptor_type);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

    double keypoint_matching_time_ms{};
    if (dataBuffer.size() > 1) // wait until at least two images have been processed
    {

      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;

      //// STUDENT ASSIGNMENT
      //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
      keypoint_matching_time_ms = matchDescriptors((dataBuffer.end() - 2)->keypoints,
                                                   (dataBuffer.end() - 1)->keypoints,
                                                   (dataBuffer.end() - 2)->descriptors,
                                                   (dataBuffer.end() - 1)->descriptors,
                                                   matches,
                                                   matcher_type,
                                                   selector_type);

      //// EOF STUDENT ASSIGNMENT

      // store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

      // visualize matches between current and previous image
      if (is_visualization_on) {
        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                        matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cout << "Press key to continue to next image" << endl;
        cv::waitKey(0); // wait for key to be pressed
      }
    }

    // count the number of keypoints on the preceding vehicle for all 10 images
    if (is_focus_on_leading_vehicle) {
      KeypointStatisticsType statistics;

      float keypoint_size_mean{};
      float keypoint_size_variance{};
      if (keypoints.size() > 0) {
        for (auto &keypoint : keypoints) {
          keypoint_size_mean += keypoint.size;
        }
        keypoint_size_mean /= keypoints.size();

        for (auto &keypoint : keypoints) {
          keypoint_size_variance += (keypoint.size - keypoint_size_mean) * (keypoint.size - keypoint_size_mean);
        }
        keypoint_size_variance /= keypoints.size();
      } else {
        keypoint_size_mean = -1;
        keypoint_size_variance = -1;
      }

      statistics.number_keypoints_on_preceding_vehicle = keypoints.size();
      statistics.size_mean = keypoint_size_mean;
      statistics.size_standard_deviation = (keypoint_size_variance >= 0) ? (std::sqrt(keypoint_size_variance)) : -1;
      statistics.number_keypoint_matches = (dataBuffer.end() - 1)->kptMatches.size();
      statistics.keypoint_detection_time_ms = static_cast<float>(detector_execution_time_ms);
      statistics.descriptor_extraction_time_ms = static_cast<float>(descriptor_extraction_time_ms);
      statistics.keypoint_matching_time_ms = static_cast<float>(keypoint_matching_time_ms);
      keypoints_statistics.push_back(statistics);
    }

  } // eof loop over all images

  std::string detector_type_str{};
  switch (detector_type) {
    case DetectorType::AKAZE: {
      detector_type_str = "AKAZE";
      break;
    }
    case DetectorType::BRISK: {
      detector_type_str = "BRISK";
      break;
    }
    case DetectorType::FAST: {
      detector_type_str = "FAST";
      break;
    }
    case DetectorType::HARRIS: {
      detector_type_str = "HARRIS";
      break;
    }
    case DetectorType::ORB: {
      detector_type_str = "ORB";
      break;
    }
    case DetectorType::SIFT: {
      detector_type_str = "SIFT";
      break;
    }
    case DetectorType::SHITOMASI: {
      detector_type_str = "SHITOMASI";
      break;
    }
    default:break;
  }

  std::string descriptor_type_str{};
  switch (descriptor_type) {
    case DescriptorType::AKAZE: {
      descriptor_type_str = "AKAZE";
      break;
    }
    case DescriptorType::BRIEF: {
      descriptor_type_str = "BRIEF";
      break;
    }
    case DescriptorType::BRISK: {
      descriptor_type_str = "BRISK";
      break;
    }
    case DescriptorType::FREAK: {
      descriptor_type_str = "FREAK";
      break;
    }
    case DescriptorType::ORB: {
      descriptor_type_str = "ORB";
      break;
    }
    case DescriptorType::SIFT: {
      descriptor_type_str = "SIFT";
      break;
    }
    default:break;
  }

  std::string matcher_type_str{};
  switch (matcher_type) {
    case MatcherType::MAT_BF: {
      matcher_type_str = "Brute Force";
      break;
    }
    case MatcherType::MAT_FLANN: {
      matcher_type_str = "FLANN";
      break;
    }
  }

  std::string selector_type_str{};
  switch (selector_type) {
    case SelectorType::SEL_NN: {
      selector_type_str = "Nearest Neighbor (best match)";
      break;
    }
    case SelectorType::SEL_KNN: {
      selector_type_str = "k Nearest Neighbors (k=2)";
      break;
    }
  }

  int avg_number_keypoints_on_preceding_vehicle{};
  float avg_size_mean{};
  float avg_size_standard_deviation{};
  int avg_number_keypoint_matches{};
  float avg_keypoint_detection_time_ms{};
  float avg_descriptor_extraction_time_ms{};
  float avg_keypoint_matching_time_ms{};
  for (auto& statistic : keypoints_statistics) {
    avg_number_keypoints_on_preceding_vehicle += statistic.number_keypoints_on_preceding_vehicle;
    avg_size_mean += statistic.size_mean;
    avg_size_standard_deviation += statistic.size_standard_deviation;
    avg_number_keypoint_matches += statistic.number_keypoint_matches;
    avg_keypoint_detection_time_ms += statistic.keypoint_detection_time_ms;
    avg_descriptor_extraction_time_ms += statistic.descriptor_extraction_time_ms;
    avg_keypoint_matching_time_ms += statistic.keypoint_matching_time_ms;
  }
  avg_number_keypoints_on_preceding_vehicle /= keypoints_statistics.size();
  avg_size_mean /= keypoints_statistics.size();
  avg_size_standard_deviation /= keypoints_statistics.size();
  avg_number_keypoint_matches /= static_cast<int>((keypoints_statistics.size() - 1));
  avg_keypoint_detection_time_ms /= keypoints_statistics.size();
  avg_descriptor_extraction_time_ms /= keypoints_statistics.size();
  avg_keypoint_matching_time_ms /= keypoints_statistics.size();

  std::cout << endl;
  std::cout << "============================================" << std::endl;
  std::cout << "================ Statistics ================" << std::endl;
  std::cout << "============================================" << std::endl;
  std::cout << endl;
  std::cout << "Detector type: " << detector_type_str << std::endl;
  std::cout << "Descriptor type: " << descriptor_type_str << std::endl;
  std::cout << "Matcher type: " << matcher_type_str << std::endl;
  std::cout << "Selector type: " << selector_type_str << std::endl;
  std::cout << "--------------------------------------------" << std::endl;
  std::cout << "Average number of keypoints on preceding vehicle: " << avg_number_keypoints_on_preceding_vehicle << std::endl;
  std::cout << "Average mean of keypoints size: " << avg_size_mean << std::endl;
  std::cout << "Average std(keypoints size) in all images: " << avg_size_standard_deviation << std::endl;
  std::cout << "Average number of keypoint matches in all images: " << avg_number_keypoint_matches << std::endl;
  std::cout << "Average time for keypoints detection in all images: " << avg_keypoint_detection_time_ms << " ms" << std::endl;
  std::cout << "Average time for descriptor extraction in all images: " << avg_descriptor_extraction_time_ms << " ms" << std::endl;
  std::cout << "Average time to find matches between 2 frames: " << avg_keypoint_matching_time_ms << " ms" << std::endl;

  return 0;
}
