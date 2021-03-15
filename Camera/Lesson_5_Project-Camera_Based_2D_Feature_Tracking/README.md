# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

In particular, the following points has been addressed:

#### MP.1 Data Buffer Optimization: 
A buffer for DataFrame objects with a fixed size has been implemented. Using std::deque, the buffer is filled by pushing back the elements into the buffer until the buffer size has reached a specified maximum. In case the buffer size has reached the maximum, the oldest element is removed at the front before a new element is pushed back into the buffer.

#### MP.2 Keypoint Detection
Different detectors such as HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT have been implemented using corresponding open cv classes. Each detector can be selected via an enum class.

#### MP.3 Keypoint Removal
All keypoints outside of a pre-defined rectangle have been removed from the image and only the keypoints within the rectangle were used for further processing. The rectangle has been defined by its top-left corner's position, its width, and height by using cv::Rect.

#### MP.4 Keypoint Descriptors
Different descriptors such as BRIEF, ORB, FREAK, AKAZE and SIFT have been implemented using corresponding open cv classes. Each descriptor can be selected via an enum class.

#### MP.5 Descriptor Matching
Two types of matching have been implemented using corresponding open cv classes: Brute force approach and FLANN matching along with the k-nearest neighbor selection.

#### MP.6 Descriptor Distance Ratio
A descriptor distance ratio test has been implemented using the K-Nearest-Neighbor matching with k = 2. The test looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

#### MP.7 Performance Evaluation 1
For every implemented detector, an average number of keypoints, mean and standard deviation of keypoints size on a preceding vehicle based on all 10 images has been calculated.

#### MP.8 Performance Evaluation 2
For every possible detector / descriptor combination, an average number of keypoint matches in all 10 images has been calculated by using the brute force approach with the descriptor distance ratio set to 0.8.

#### MP.9 Performance Evaluation 3
Time logging has been implemented for keypoint detection and descriptor extraction. All retrieved results have been entered into a spreadsheet and based on this data, the TOP3 detector/descriptor combinations have been recommended as the best choice for our purpose of detecting keypoints on vehicles.


## Dependencies for Running Locally
* see [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion/tree/main/Camera#b-dependencies-for-running-locally) for details.

## Basic Build Instructions

1. Clone the entire repo as explained [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion#cloning)
2. Change directory to UdacityNanoDegree-SensorFusion/Camera/Lesson_5_Project-Camera_Based_2D_Feature_Tracking
3. Make a build directory in the top level directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./2D_feature_tracking`.
