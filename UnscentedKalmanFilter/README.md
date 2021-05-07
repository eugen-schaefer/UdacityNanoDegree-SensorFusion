# SFND_Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project, an Unscented Kalman Filter has been implemented to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the following tolerance:

* RMSE for longitudinal position <= 0.30 m
* RMSE for lateral position <= 0.16 m
* RMSE for longitudinal velocity <= 0.95 m
* RMSE for lateral velocity <= 0.70 m


<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has it's own UKF object generated for it, and will update each indidual one during every time step.

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Dependencies for Running Locally
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2 + Eigen
    * sudo apt install libpcl-dev

---

## Build & Run Instructions

1. Clone the entire repo as explained [HERE](https://github.com/eugen-schaefer/UdacityNanoDegree-SensorFusion#cloning)
2. Change directory to UdacityNanoDegree-SensorFusion/UnscentedKalmanFilter/
3. Make a build directory in the Lidar directory and change into it: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./ukf_highway`