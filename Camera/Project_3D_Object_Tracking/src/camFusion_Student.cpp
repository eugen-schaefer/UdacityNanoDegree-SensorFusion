
#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

template <typename T>
T CalculateMedian(vector<T> &input_list) {
  T median{};
  if (!input_list.empty()) {
    std::sort(input_list.begin(), input_list.end());
    int size = input_list.size();
    if (size % 2 == 0) {
      median = (input_list[size / 2 - 1] + input_list[size / 2]) / 2;
    } else {
      median = input_list[size / 2];
    }
  }
  return median;
}

template <typename T>
T CalculateMean(vector<T> &input_list) {
  return (std::accumulate(input_list.begin(), input_list.end(), 0.0f)) /
         (static_cast<T>(input_list.size()));
}

template <typename T>
T CalculateMinimum(vector<T> &input_list) {
  T min_value{std::numeric_limits<T>::max()};
  for (auto &input : input_list) {
    min_value = input < min_value ? input : min_value;
  }
  return min_value;
}

// Create groups of Lidar points whose projection into the camera falls into the
// same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes,
                         std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor, cv::Mat &P_rect_xx,
                         cv::Mat &R_rect_xx, cv::Mat &RT) {
  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    // pixel coordinates
    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

    // pointers to all bounding boxes which enclose the current Lidar point
    vector<vector<BoundingBox>::iterator> enclosingBoxes;

    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin();
         it2 != boundingBoxes.end(); ++it2) {
      // shrink current bounding box slightly to avoid having too many outlier
      // points around the edges
      cv::Rect smallerBox;
      smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
      smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
      smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
      smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

      // check wether point is within current bounding box
      if (smallerBox.contains(pt)) {
        enclosingBoxes.push_back(it2);
      }

    }  // eof loop over all bounding boxes

    // check wether point has been enclosed by one or by multiple boxes
    if (enclosingBoxes.size() == 1) {
      // add Lidar point to bounding box
      enclosingBoxes[0]->lidarPoints.push_back(*it1);
    }

  }  // eof loop over all Lidar points
}

/*
 * The show3DObjects() function below can handle different output image sizes,
 * but the text output has been manually tuned to fit the 2000x2000 size.
 * However, you can make this function work for other sizes too.
 * For instance, to use a 1000x1000 size, adjusting the text positions by
 * dividing them by 2.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize,
                   cv::Size imageSize, bool bWait) {
  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

  for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
    // create randomized color for current 3D object
    cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150),
                                      rng.uniform(0, 150));

    // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end();
         ++it2) {
      // world coordinates
      float xw =
          (*it2).x;  // world position in m with x facing forward from sensor
      float yw =
          (*it2).y;  // world position in m with y facing left from sensor
      xwmin = xwmin < xw ? xwmin : xw;
      ywmin = ywmin < yw ? ywmin : yw;
      ywmax = ywmax > yw ? ywmax : yw;

      // top-view coordinates
      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // find enclosing rectangle
      top = top < y ? top : y;
      left = left < x ? left : x;
      bottom = bottom > y ? bottom : y;
      right = right > x ? right : x;

      // draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),
                  cv::Scalar(0, 0, 0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50),
            cv::FONT_ITALIC, 2, currColor);
    sprintf(str2, "x_min=%2.2f m, y_width=%2.2f m", xwmin, ywmax - ywmin);
    putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125),
            cv::FONT_ITALIC, 2, currColor);
  }

  // plot distance markers
  float lineSpacing = 2.0;  // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) +
            imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y),
             cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "3D Objects";
  cv::namedWindow(windowName, cv::WINDOW_NORMAL);
  cv::imshow(windowName, topviewImg);

  if (bWait) {
    cv::waitKey(0);  // wait for key to be pressed
  }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &bounding_box,
                              std::vector<cv::KeyPoint> &kpts_prev,
                              std::vector<cv::KeyPoint> &kpts_curr,
                              std::vector<cv::DMatch> &kpt_matches) {
  // First, assign keypoint matches inside the ROI
  for (auto const &match : kpt_matches) {
    bool is_prev_kp_in_bb{
        bounding_box.roi.contains(kpts_prev[match.queryIdx].pt)};
    bool is_curr_kp_in_bb{
        bounding_box.roi.contains(kpts_curr[match.trainIdx].pt)};
    if (is_prev_kp_in_bb && is_curr_kp_in_bb) {
      bounding_box.kptMatches.push_back(match);
    }
  }

  // Next, compute a robust median (median) of all the euclidean distances
  // between keypoint matches inside the bounding box
  std::vector<float> euclidean_dist{};
  std::vector<std::vector<cv::DMatch>::iterator> it_vec;
  for (auto &match : bounding_box.kptMatches) {
    float prev_pos_x = kpts_prev[match.queryIdx].pt.x;
    float prev_pos_y = kpts_prev[match.queryIdx].pt.y;
    float curr_pos_x = kpts_curr[match.trainIdx].pt.x;
    float curr_pos_y = kpts_curr[match.trainIdx].pt.y;
    euclidean_dist.push_back(
        std::sqrt((prev_pos_x - curr_pos_x) * (prev_pos_x - curr_pos_x) +
                  (prev_pos_y - curr_pos_y) * (prev_pos_y - curr_pos_y)));
  }
  std::sort(euclidean_dist.begin(), euclidean_dist.end());
  float median{CalculateMedian<float>(euclidean_dist)};

  // Finally, remove the keypoint matches that are too far away from the median
  float threshold{1};
  for (auto it = bounding_box.kptMatches.begin();
       it != bounding_box.kptMatches.end();) {
    float prev_pos_x = kpts_prev[it->queryIdx].pt.x;
    float prev_pos_y = kpts_prev[it->queryIdx].pt.y;
    float curr_pos_x = kpts_curr[it->trainIdx].pt.x;
    float curr_pos_y = kpts_curr[it->trainIdx].pt.y;
    float dist =
        std::sqrt((prev_pos_x - curr_pos_x) * (prev_pos_x - curr_pos_x) +
                  (prev_pos_y - curr_pos_y) * (prev_pos_y - curr_pos_y));
    if (std::abs(dist - median) > threshold) {
      it = bounding_box.kptMatches.erase(it);
    } else {
      ++it;
    }
  }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in
// successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate,
                      double &TTC, cv::Mat *visImg) {
  // compute distance ratios between all matched keypoints
  vector<double> distRatios;  // stores the distance ratios for all keypoints
                              // between curr. and prev. frame
  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1;
       ++it1) {  // outer keypoint loop

    // get current keypoint and its matched partner in the prev. frame
    cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
    cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end();
         ++it2) {  // inner keypoint loop

      double minDist = 100.0;  // min. required distance

      // get next keypoint and its matched partner in the prev. frame
      cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
      cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

      // compute distances and distance ratios
      double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
      double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

      if (distPrev > std::numeric_limits<double>::epsilon() &&
          distCurr >= minDist) {  // avoid division by zero

        double distRatio = distCurr / distPrev;
        distRatios.push_back(distRatio);
      }
    }  // eof inner loop over all matched kpts
  }    // eof outer loop over all matched kpts

  // only continue if list of distance ratios is not empty
  if (distRatios.empty()) {
    TTC = NAN;
    return;
  }

  // compute camera-based TTC from distance ratios
  double dT = 1 / frameRate;
  // TTC = -dT / (1 - CalculateMean<double>(distRatios));
  TTC = -dT / (1 - CalculateMedian<double>(distRatios));
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate,
                     double &TTC) {
  // Sort the lidar points in ascending order w.r.t. x-coordinate to ensure the
  // point correspondence between 2 frames.
  std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(),
            [](LidarPoint a, LidarPoint b) { return (a.x < b.x); });
  std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(),
            [](LidarPoint a, LidarPoint b) { return (a.x < b.x); });

  int nr_points_to_consider{
      static_cast<int>(min(lidarPointsPrev.size(), lidarPointsPrev.size()))};

  std::vector<float> ttc_list{};
  double reflectiveness_threshold{0.0};
  for (int index = 0; index < nr_points_to_consider; ++index) {
    if (lidarPointsCurr[index].r > reflectiveness_threshold) {
      ttc_list.push_back(1.0 / frameRate * lidarPointsCurr[index].x /
                         (lidarPointsPrev[index].x - lidarPointsCurr[index].x));
    }
  }

  // Calculate the mean/median out of all entries in the TTC list
  if (!ttc_list.empty()) {
    // TTC = CalculateMean<float>(ttc_list);
    TTC = CalculateMedian<float>(ttc_list);
  }
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bb_best_matches,
                        DataFrame &prev_frame, DataFrame &curr_frame) {
  // Initialize a table with zeros to hold the bounding box matches between the
  // previous and current frames. The rows corresponds to bounding box indices
  // from the previous frame and the columns to to bounding box indices from the
  // current frame.
  int nr_bb_in_prev_frame = prev_frame.boundingBoxes.size();
  int nr_bb_in_curr_frame = curr_frame.boundingBoxes.size();
  int nr_bb_matches[nr_bb_in_prev_frame][nr_bb_in_curr_frame];
  memset(nr_bb_matches, 0, sizeof(nr_bb_matches));

  // Iterate over all keypoint matches and fill the table with numbers counting
  // how many matches the corresponding bounding box pair have had
  for (auto const &match : matches) {
    auto prev_keypoint = prev_frame.keypoints[match.queryIdx].pt;
    auto curr_keypoint = curr_frame.keypoints[match.trainIdx].pt;

    for (int i = 0; i < nr_bb_in_prev_frame; ++i) {
      bool is_prev_keypoint_in_prev_bb{
          prev_frame.boundingBoxes[i].roi.contains(prev_keypoint)};
      for (int j = 0; j < nr_bb_in_curr_frame; ++j) {
        bool is_curr_keypoint_in_prev_bb{
            curr_frame.boundingBoxes[j].roi.contains(curr_keypoint)};
        if (is_prev_keypoint_in_prev_bb && is_curr_keypoint_in_prev_bb) {
          ++nr_bb_matches[i][j];
        }
      }
    }
  }

  // Find maximum in the table and assign it to the output
  int maximum_in_row{};
  int bb_idx_in_prev_frame_with_most_matches{-1};
  int bb_idx_in_curr_frame_with_most_matches{-1};
  for (int i = 0; i < nr_bb_in_prev_frame; i++) {
    maximum_in_row = 0;
    for (int j = 0; j < nr_bb_in_curr_frame; j++) {
      if (nr_bb_matches[i][j] > maximum_in_row) {
        maximum_in_row = nr_bb_matches[i][j];
        bb_idx_in_prev_frame_with_most_matches = i;
        bb_idx_in_curr_frame_with_most_matches = j;
      }
    }
    if (maximum_in_row > 0) {
      auto bb_id_in_prev_frame{
          prev_frame.boundingBoxes[bb_idx_in_prev_frame_with_most_matches]
              .boxID};
      auto bb_id_in_curr_frame{
          curr_frame.boundingBoxes[bb_idx_in_curr_frame_with_most_matches]
              .boxID};
      bb_best_matches.insert({bb_id_in_prev_frame, bb_id_in_curr_frame});
    }
  }
}
