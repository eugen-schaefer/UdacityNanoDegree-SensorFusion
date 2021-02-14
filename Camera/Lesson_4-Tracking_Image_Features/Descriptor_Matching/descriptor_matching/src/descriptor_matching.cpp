#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "structIO.hpp"

using namespace std;

void matchDescriptors(cv::Mat &imgSource,
                      cv::Mat &imgRef,
                      vector<cv::KeyPoint> &kPtsSource,
                      vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      vector<cv::DMatch> &matches,
                      string descriptorType,
                      string matcherType,
                      string selectorType) {

  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType == "MAT_BF") {

    int normType = descriptorType == "DES_BINARY" ? cv::NORM_HAMMING : cv::NORM_L2;
    matcher = cv::BFMatcher::create(normType, crossCheck);
    cout << "BF matching cross-check=" << crossCheck;
  } else if (matcherType == "MAT_FLANN") {
    if (descSource.type()
        != CV_32F) { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
      descSource.convertTo(descSource, CV_32F);
      descRef.convertTo(descRef, CV_32F);
    }

    //... TODO : implement FLANN matching
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    cout << "FLANN matching";
  }

  // perform matching task
  if (selectorType == "SEL_NN") { // nearest neighbor (best match)

    auto t = (double) cv::getTickCount();
    matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
    cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
  } else if (selectorType == "SEL_KNN") { // k nearest neighbors (k=2)

    // TODO : implement k-nearest-neighbor matching
    std::vector<std::vector<cv::DMatch>> knn_matches;
    int k = 2;
    auto t = (double) cv::getTickCount();
    matcher->knnMatch(descSource, descRef, knn_matches, k);

    // TODO : filter matches using descriptor distance ratio test
    float dist_ratio_threshold{0.8f};
    for (auto &knn_match : knn_matches) {
      if (knn_match[0].distance / knn_match[1].distance < dist_ratio_threshold) {
        matches.push_back(knn_match[0]);
      }
    }
    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
    cout << " (kNN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
  }

  // visualize results
  cv::Mat matchImg = imgRef.clone();
  cv::drawMatches(imgSource,
                  kPtsSource,
                  imgRef,
                  kPtsRef,
                  matches,
                  matchImg,
                  cv::Scalar::all(-1),
                  cv::Scalar::all(-1),
                  vector<char>(),
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  string windowName = "Matching keypoints between two camera images (best 50)";
  cv::namedWindow(windowName, 7);
  cv::imshow(windowName, matchImg);
  cv::waitKey(0);
}

int main() {
  cv::Mat imgSource = cv::imread("../images/img1gray.png");
  cv::Mat imgRef = cv::imread("../images/img2gray.png");

  vector<cv::KeyPoint> kptsSource, kptsRef;
//  readKeypoints("../dat/C35A5_KptsSource_BRISK_large.dat", kptsSource);
//  readKeypoints("../dat/C35A5_KptsRef_BRISK_large.dat", kptsRef);
  readKeypoints("../dat/C35A5_KptsSource_BRISK_small.dat", kptsSource);
  readKeypoints("../dat/C35A5_KptsRef_BRISK_small.dat", kptsRef);

  cv::Mat descSource, descRef;
//  readDescriptors("../dat/C35A5_DescSource_BRISK_large.dat", descSource);
//  readDescriptors("../dat/C35A5_DescRef_BRISK_large.dat", descRef);
  readDescriptors("../dat/C35A5_DescSource_BRISK_small.dat", descSource);
  readDescriptors("../dat/C35A5_DescRef_BRISK_small.dat", descRef);

  vector<cv::DMatch> matches;
  string matcherType = "MAT_BF";
  string descriptorType = "DES_BINARY";
  string selectorType = "SEL_KNN";//"SEL_NN";
  matchDescriptors(imgSource,
                   imgRef,
                   kptsSource,
                   kptsRef,
                   descSource,
                   descRef,
                   matches,
                   descriptorType,
                   matcherType,
                   selectorType);
}