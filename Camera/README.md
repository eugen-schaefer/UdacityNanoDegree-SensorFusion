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
     - Useful links:
        - https://www.allaboutcircuits.com/technical-articles/two-dimensional-convolution-in-image-processing/
        - http://songho.ca/dsp/convolution/convolution.html
        - https://computergraphics.stackexchange.com/questions/39/how-is-gaussian-blur-implemented
       
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
  * Installation steps (inspired from [here](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html), [here](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/#installing-opencv-from-the-source) and [here](https://learnopencv.com/install-opencv-4-on-ubuntu-18-04/)):
    * #### change into some directory, e.g. Downloads/
      `cd ~/Downloads/`
    * #### Download and unpack sources for OpenCV
      `wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip`
      `unzip opencv.zip`
    * #### Download and unpack sources for OpenCV's extra modules
      `wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip`
      `unzip opencv_contrib.zip`
    * #### Build OpenCV, such that it will include the extra modules
      `cd opencv-master`
      
      `mkdir build && cd build`
      
      `cmake -D OPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-master/modules ..`
      
      `make -j<n>` # --> n is the number of processor kernels, you can check it via `nproc
    * #### Install OpenCV
      `sudo make install`
  

  * Compiling from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag enables using SIFT and SURF detectors.
  * For general information about openCV see [here](https://github.com/opencv/opencv/tree/4.1.0).


## C. Build Instructions
1. Clone the entire repo as explained [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion#cloning)
2. Change directory to UdacityNanoDegree-SensorFusion/Camera/
3. Go to the top level directory for an exercise where you see src folder and CMakeLists.txt, and run the following commands on your terminal:
```
mkdir build && cd build
cmake ..
make
./<Executable_File_Name>
``` 
	
