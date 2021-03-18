# 3D Object Tracking

This is the final project of the camera course. By completing all the previous lessons, the following knowledge has been gained:

 - solid understanding of keypoint detectors, descriptors, and methods to match them between successive images
 - detect objects in an image using the YOLO deep-learning framework
 - how to associate regions in a camera image with Lidar points in 3D space
 
The following program schematic depicts what already has been accomplished and what's still missing and hence the content of the final project.

<p align="center">
<img src="images/course_code_structure.png" width="779" height="414" />
</p>


In this final project, the missing parts in the schematic have been implemented. In detail, there have been the following tasks:
 
1. Develop a way to match 3D objects over time by using keypoint correspondences.
2. Compute the TTC based on Lidar measurements. 
3. Compute the TTC based on camera measurements, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. Conduct various tests with the framework. The goal was to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor.

### 4.1 Performance evaluation for Lidar-based TTC

<p align="center">
<img src="images/Illustration_TTC_Lidar.png"/>
</p>

The TTC calculation for Lidar is based on the constant-velocity model. The velocity `v_0` can be computed from two successive Lidar measurements as follows:

<p align="center">
<img src="images/TTC_formula_lidar.png" width="300"/>
</p>

So given a Lidar sensor that is able to make precise distance measurements, a system for TTC estimation can be developed based on a constant-velocity model and on the set of equations shown above.

In order to compute the TTC, we need to find the distance to the closest Lidar point of the preceding vehicle in the path of driving. After postprocessing of the Lidar points and projecting them into the camera image we limit the lidar points to the preceding vehicle. The majority of the Lidar points on the tailgate of the preceding vehicle is a good candidate for the TTC calculation. However, taking the closest point in the driving direction, i.e. min(Lidarpoint.positionX), might be very error-prone because of some outliers, faulty measurements, etc.

So one idea is to take the arithmetic mean out of all lidar points longitudinal components. This way we observe some faulty TTC estimates (Inf) by looking at some frames (e.g. frame 2, 24, 36).


<p align="center">
<table style="width: 100%;"><tbody><tr>
<img src="images/TTC_Lidar_NAN_Frame2.png" width=33%/>
<img src="images/TTC_Lidar_NAN_Frame24.png" width=33%/> 
<img src="images/TTC_Lidar_NAN_Frame36.png" width=33%/>
</tr></tbody></table>
</p>

From debugging in the code it can be seen that faulty estimates result from the same mean between consecutive frames, so d_0 and d_1 are sometimes equal that yields a zero denominator resulting in erroneous estimation of TTC. 

Taking the median instead of the mean out of all lidar points longitudinal components makes the estimation a bit more robust such that the above-mentioned errors can be mitigated.

<p align="center">
<table style="width: 100%;"><tbody><tr>
<img src="images/TTC_Lidar_notNAN_Frame2.png" width=33%/>
<img src="images/TTC_Lidar_notNAN_Frame24.png" width=33%/> 
<img src="images/TTC_Lidar_notNAN_Frame36.png" width=33%/>
</tr></tbody></table>
</p>

The following chart depicts the estimated TTC over all frames using both, median and mean to select the significant lidar point. Until frame 53, median and mean variants show similar results for TTC, something around 9 seconds. The estimation becomes more unstable with increasing frames. The reason is that the relative velocity between the ego and the preceding vehicle becomes smaller and smaller and in the end, it approaches zero.

<p align="center">
<img src="images/Chart_Lidar_TTC_over_Frames.png"/>   
</p>


### 4.2 Performance evaluation for Camera-based TTC

Monocular cameras are not able to measure metric distances. They are passive sensors that rely on the ambient light which reflects off of objects into the camera lens. It is thus not possible to measure the runtime of light as with Lidar technology.

To measure distance, a second camera would be needed. Given two images taken by two carefully aligned cameras (also called a stereo setup) at the same time instant, one would have to locate common points of interest in both images (e.g. the tail lights of the preceding vehicle) and then triangulate their distance using camera geometry and perspective projection. 

In case of mono vision camera the TTC is calculated based on the pixel distances on the image plane by using the constant velocity motion model. 

In the following figure, you can see how the height H of the preceding vehicle can be mapped onto the image plane using perspective projection. We can see that the same height H maps to different heights h0 and h1 in the image plane, depending on the distance d0 and d1 of the vehicle. There is a geometric relation between h, H, d and the focal length f of the pinhole camera - and this is exploited to calculate TTC.

<p align="center">
<img src="images/Illustration_TTC_Camera.png"/>
</p>

<p align="center">
  <img width="500" src="images/TTC_formula_camera.png">
</p>

Since we don't measure the vehicle's height directly, we rather utilize the keypoint distances between successive frames. The following figure illustrates the concept:

<p align="center">
<img src="images/keypoint_distances.png" width="500" />
</p>

In (a), a set of keypoints has been detected and the relative distances between keypoints 1-7 have been computed. In (b), 4 keypoints have been matched between successive images (with keypoint 3 being a mismatch) using a higher-dimensional similarity measure called descriptor. The ratio of all relative distances between each other can be used to compute a reliable TTC estimate by replacing the height ratio h1/h0 with the mean or median of all distance ratios d_k / d_k'd.

The following tables depict various detector/descriptor combinations w.r.t. their performance regarding TTC estimation.  

<p align="center">
  <img src="images/CAM_DATA_AKAZE_FAST.png">
</p>

<p align="center">
  <img src="images/CAM_DATA_HARRIS_SIFT.png">
</p>

Red-colored cells indicate TTC estimates with absolute values greater than 50 s and thus considered as outliers. 

As the following chart shows, TTC estimation is getting unstable approximately from frame 50 for all detector/descriptor combinations. The reason for this instability is the fact that we are approaching a red light at an intersection and the distance between ego and the preceding vehicle becomes constant between consecutive frames. Considering the formula for camera-based TTC calculation, the fraction h1/h0 approaches the value 1 and hence the denominator of the TTC formula tends to zero resulting in inflated TTC estimates.

We also see that HARRIS and ORB detectors have many outliers in the first 50 frames. 

<p align="center">
  <img src="images/Chart_Detector_Descriptor_Eval_All.png">
</p>

The following charts depict detector/desciptor combinations showing more or less a stable TTC estimation.

<p align="center">
<table style="width: 100%;"><tbody><tr>
<img src="images/Chart_AKAZE_BRISK.png" width=50%/>
<img src="images/Chart_Shitomasi_SIFT.png" width=50%/> 
</tr></tbody></table>
</p>

## Dependencies for Running Locally
* see [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion/tree/main/Lidar#dependencies-for-running-locally-in-ubuntu) and [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion/tree/main/Camera#b-dependencies-for-running-locally) for details.


## Basic Build Instructions

1. Clone the entire repo as explained [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion#cloning)
2. Change directory to UdacityNanoDegree-SensorFusion/Camera/Project_3D_Object_Tracking
3. Make a build directory in the top level directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./3D_object_tracking`
