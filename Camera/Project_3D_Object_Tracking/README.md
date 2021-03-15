# 3D Object Tracking

This is the final project of the camera course. By completing all the previous lessons, the following knowledge has been gained:

 - solid understanding of keypoint detectors, descriptors, and methods to match them between successive images
 - detect objects in an image using the YOLO deep-learning framework
 - how to associate regions in a camera image with Lidar points in 3D space
 
The following program schematic depicts what already has been accomplished and what's still missing and hence the content of the final project.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, the missing parts in the schematic have been implemented. In detail, there were the following tasks:
 
1. Develop a way to match 3D objects over time by using keypoint correspondences.
2. Compute the TTC based on Lidar measurements. 
3. Compute the TTC based on camera measurements, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. Conduct various tests with the framework. The goal was to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor.

### 4.1 Performance evaluation for Lidar-based TTC

<img src="images/Illustration_TTC_Lidar.png"/>

The calculation TTC for Lidar is based on the constant-velocity model. The velocity `v_0` can be computed from two successive Lidar measurements as follows:

<img src="images/TTC_formula_lidar.png"/>

So given a Lidar sensor which is able to take precise distance measurements, a system for TTC estimation can be developed based on a constant-velocity model and on the set of equations shown above.

In order to compute the TTC, we need to find the distance to the closest Lidar point of the preceding vehicle in the path of driving. After postprocessing of the Lidar points and projecting them into the camera image we limit the lidar points to the preceding vehicle. The majority of the Lidar points on the tailgate of the preceding vehicle is a good candidate for the TTC calculation. However, taking the closest point in the driving direction, i.e. min(Lidarpoint.positionX), might be very error-prone because of some outliers, faulty measurements, etc.

So one idea is to take the arithmetic mean out of all lidar points longitudinal components. This way we ebserve some faulty estimates of the TTC (Inf) by looking at some frames (e.g. frame 2, 24, 36)

<p float="left">
  <img src="images/TTC_Lidar_NAN_Frame2.png" width="260" />
  <img src="images/TTC_Lidar_NAN_Frame24.png" width="260" /> 
  <img src="images/TTC_Lidar_NAN_Frame36.png" width="260" />
</p>

From debugging in the code it can be seen that faulty estimates result from the same mean between consecutive frames, so d_0 and d_1 are sometimes equal that yields a zero denominator resulting in erroneous estimation of TTC. 

Taking the median instead of the mean out of all lidar points longitudinal components makes the estimation a bit more robust such that the above-mentioned errors can be mitigated.

<p float="left">
  <img src="images/TTC_Lidar_notNAN_Frame2.png" width="260" />
  <img src="images/TTC_Lidar_notNAN_Frame24.png" width="260" /> 
  <img src="images/TTC_Lidar_notNAN_Frame36.png" width="260" />
</p>

The following chart depicts the estimated TTC over all frames using both, median and mean to select the significant lidar point. Until frame 53, median and mean variants show similar results for TTC, something around 9 seconds. The estimation becomes more unstable with increasing frames. The reason is that the relative velocity between the ego and the preceding vehicle becomes smaller and smaller and in the end, it approaches zero.

<img src="images/TTC_over_Frames.png"/>   



### 4.2 Performance evaluation for Camera-based TTC

t.b.d.


## Dependencies for Running Locally
* see [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion/tree/main/Lidar#dependencies-for-running-locally-in-ubuntu) and [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion/tree/main/Camera#b-dependencies-for-running-locally) for details.


## Basic Build Instructions

1. Clone the entire repo as explained [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion#cloning)
2. Change directory to UdacityNanoDegree-SensorFusion/Camera/Project_3D_Object_Tracking
3. Make a build directory in the top level directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./3D_object_tracking`
