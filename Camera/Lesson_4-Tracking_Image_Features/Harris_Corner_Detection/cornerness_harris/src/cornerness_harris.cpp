#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void CalculateMaximumWithinWindow(cv::Mat &image_matrix,
                                  vector<cv::KeyPoint> &key_points,
                                  int left_upper_x,
                                  int left_upper_y,
                                  int width,
                                  int height) {
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;

  cv::Mat window(image_matrix, cv::Rect(left_upper_x, left_upper_y, width, height));
  minMaxLoc(window, &minVal, &maxVal, &minLoc, &maxLoc);
  cv::KeyPoint kp{};
  kp.pt = {static_cast<float>(maxLoc.x), static_cast<float>(maxLoc.y)};
  kp.response = maxVal;
  kp.size = 2.0;
  key_points.push_back(kp);
}

void cornernessHarris() {
  // load image from file
  cv::Mat img;
  img = cv::imread("../images/img1.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);  // convert to grayscale

  // Detector parameters
  int blockSize =
      2;  // for every pixel, a blockSize × blockSize neighborhood is considered
  int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
  int minResponse =
      100;  // minimum value for a corner in the 8bit scaled response matrix
  double k = 0.04;  // Harris parameter (see equation for details)

  // Detect Harris corners and normalize output
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  // visualize results
  string windowName = "Harris Corner Detector Response Matrix";
  cv::namedWindow(windowName, 4);
  cv::imshow(windowName, dst_norm_scaled);
  cv::waitKey(0);


  // TODO: Your task is to locate local maxima in the Harris response matrix
  // and perform a non-maximum suppression (NMS) in a local neighborhood around
  // each maximum. The resulting coordinates shall be stored in a list of
  // keypoints of the type `vector<cv::KeyPoint>`.
  int window_x_size{100};
  int window_y_size{100};
  vector<cv::KeyPoint> key_points{};

  int dx = window_x_size - dst_norm_scaled.cols % window_x_size;
  int dy = window_y_size - dst_norm_scaled.rows % window_y_size;

  cv::Mat dst_norm_scaled_zeropadded(dst_norm_scaled.rows + dy, dst_norm_scaled.cols + dx, CV_8UC1, cv::Scalar(0));

  cv::Range src_row_range = cv::Range(0, dst_norm_scaled.rows); //select maximum allowed cols
  cv::Range src_col_range = cv::Range(0, dst_norm_scaled.cols); //select maximum allowed cols
  dst_norm_scaled(src_row_range, src_col_range).copyTo(dst_norm_scaled_zeropadded(src_row_range, src_col_range));

  for (int r = 0; r < dst_norm_scaled_zeropadded.rows - window_y_size; r += window_y_size) {
    for (int c = 0; c < dst_norm_scaled_zeropadded.cols - window_x_size; c += window_x_size) {
      CalculateMaximumWithinWindow(dst_norm_scaled_zeropadded, key_points, c, r, window_x_size, window_y_size);
    }
  }

  windowName = "Harris Corner Detection Results";
  cv::namedWindow(windowName, 5);
  cv::Mat visImage = dst_norm_scaled.clone();
  cv::drawKeypoints(dst_norm_scaled, key_points, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow(windowName, visImage);
  cv::waitKey(0);

}

int main() {
  cornernessHarris();
}