# Sensor Fusion NanoDegree- Camera Course
Camera is the second course in the Sensor Fusion ND. The purpose of this repo is to provide the exercise code to the students, so that they can practice in local system. 

This repo contains lesson-wise exercises and corresponding solutions for Udacity's Sensor Fusion ND. 

## A. List of Lesson-wise Exercises
1. Lesson 2: Autonomous Vehicles and Computer Vision
   - The OpenCV Library
1. Lesson 3: Engineering a Collision Detection System
   - Estimating TTC with Camera
   - Estimating TTC with Lidar
1. Lesson 4: Tracking Image Features
   - Descriptor Matching
   - Gradient-based vs. Binary Descriptors
   - Haris Corner Detection
   - Intensity Gradient and Filtering
   - Overview of Popular Keypoint Detectors
1. Lesson 5: Starter code for "Project: Camera Based 2D Feature Tracking" is available here - https://github.com/udacity/SFND_2D_Feature_Tracking
1. Lesson 6: Combining Camera and Lidar
   - Creating 3D-Objects
   - Lidar-to-Camera Point Projection
   - Object Detection with YOLO
1. Lesson 7: Starter code for "Project: Track an Object in 3D Space" is available here - https://github.com/udacity/SFND_3D_Object_Tracking


## B. Dependencies for Running Locally

* sudo apt install build-essential
  * includes the GCC/g++ compilers and libraries and some other utilities.
* cmake >= 3.18
  * [click here for installation instructions](https://cmake.org/install/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)


## C. Build Instructions
1. Clone the entire repo as explained [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion#cloning)
2. Change directory to UdacityNanoDegree-SensorFusion/Camera/
3. Go to the top level directory for an exercise, and run the following commands on your terminal:
```
mkdir build && cd build
cmake ..
make
./<Executable_File_Name>
``` 
	
