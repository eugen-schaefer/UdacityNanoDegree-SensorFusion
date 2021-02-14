#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

void gradientSobel() {
  // TODO: Based on the image gradients in both x and y, compute an image
  // which contains the gradient magnitude according to the equation at the
  // beginning of this section for every pixel position. Also, apply different
  // levels of Gaussian blurring before applying the Sobel operator and compare
  // the results. load image from file
  cv::Mat img;
  img = cv::imread("../images/img1.png");

  // convert image to grayscale
  cv::Mat imgGray;
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  // Filter out some noise in the image
  cv::Mat imgBlurred;
  GaussianBlur(imgGray, imgBlurred, Size(5, 5), 2.0, 2.0);

  // create filter kernel
  float sobel_x[9] = {-1, 0, +1, -2, 0, +2, -1, 0, +1};
  float sobel_y[9] = {-1, -2, -1, 0, 0, 0, +1, +2, +1};
  cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
  cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

  // apply filter
  cv::Mat result_x;
  cv::Mat result_y;
  cv::filter2D(imgBlurred, result_x, -1, kernel_x, cv::Point(-1, -1), 0,
               cv::BORDER_DEFAULT);
  cv::filter2D(imgBlurred, result_y, -1, kernel_y, cv::Point(-1, -1), 0,
               cv::BORDER_DEFAULT);

  cv::Mat result_xy = imgBlurred.clone();
  for (int r = 0; r < result_xy.rows; r++)
  {
    for (int c = 0; c < result_xy.cols; c++)
    {
      result_xy.at<unsigned char>(r, c) = sqrt(pow(result_x.at<unsigned char>(r, c), 2) +
          pow(result_y.at<unsigned char>(r, c), 2));
    }
  }

  // Apply Sobel operator from CV
  Mat grad, grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;
  Sobel(imgBlurred, grad_x, -1, 1, 0, 3, 1, 0, BORDER_DEFAULT);
  Sobel(imgBlurred, grad_y, -1, 0, 1, 3, 1, 0, BORDER_DEFAULT);
  // converting back to CV_8U
  convertScaleAbs(grad_x, abs_grad_x);
  convertScaleAbs(grad_y, abs_grad_y);
  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

  // Apply Laplacian operator from CV
  Mat laplace;
  Laplacian(imgBlurred, laplace, CV_32F, 3);

  // show result
  string windowName = "Sobel operator (x-direction)";
  cv::namedWindow(windowName, 1);  // create window
  cv::imshow(windowName, result_x);

  windowName = "Sobel operator (y-direction)";
  cv::namedWindow(windowName, 1);  // create window
  cv::imshow(windowName, result_y);

  windowName = "Sobel operator (xy-direction)";
  cv::namedWindow(windowName, 1);  // create window
  cv::imshow(windowName, result_xy);

  windowName = "CV::Sobel operator (xy-direction)";
  cv::namedWindow(windowName, 1);  // create window
  cv::imshow(windowName, grad);

  windowName = "CV::Laplacian operator (xy-direction)";
  cv::namedWindow(windowName, 1);  // create window
  cv::imshow(windowName, laplace);

  cv::waitKey(0);  // wait for keyboard input before continuing
}

int main() { gradientSobel(); }