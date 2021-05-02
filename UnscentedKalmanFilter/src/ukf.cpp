#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // define spreading parameter
  lambda_ = 3 - n_x_;

  // initial state vector
  x_ = Eigen::VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(n_x_, n_x_);
  P_.fill(0.0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
}

UKF::~UKF() {}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd& sigma_points){
     // create augmented mean vector
    Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);

    // create augmented state covariance
    Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);

    x_aug.head(n_x_) = x_;
    for (size_t i = n_x_; i < n_aug_; i++)
    {
      x_aug(i) = 0;
    }

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5, 5) = std_a_*std_a_;
    P_aug(6, 6) = std_yawdd_*std_yawdd_;

    // create square root matrix
    MatrixXd sqrt_P_aug = P_aug.llt().matrixL();

    // create augmented sigma points
    sigma_points.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; ++i) {
      sigma_points.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * sqrt_P_aug.col(i);
      sigma_points.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * sqrt_P_aug.col(i);
    }
    
}

void UKF::PredictAugmentedSigmaPoints(Eigen::MatrixXd& sigma_points, double delta_t){
  for (int i = 0; i< 2*n_aug_+1; ++i){
    // extract values for better readability
    double p_x = sigma_points(0,i);
    double p_y = sigma_points(1,i);
    double v = sigma_points(2,i);
    double yaw = sigma_points(3,i);
    double yaw_rate = sigma_points(4,i);
    double translational_acceleration_noise = sigma_points(5,i);
    double rotational_acceleration_noise = sigma_points(6,i);

    // avoid division by zero
    if (fabs(yaw_rate) > 0.001) {
        p_x = p_x + v/yaw_rate * ( sin (yaw + yaw_rate*delta_t) - sin(yaw));
        p_y = p_y + v/yaw_rate * ( cos(yaw) - cos(yaw+yaw_rate*delta_t) );
    } else {
        p_x = p_x + v*delta_t*cos(yaw);
        p_y = p_y + v*delta_t*sin(yaw);
    }

    yaw = yaw + yaw_rate*delta_t;

    // add noise
    p_x = p_x + 0.5*translational_acceleration_noise*delta_t*delta_t * cos(yaw);
    p_y = p_y + 0.5*translational_acceleration_noise*delta_t*delta_t * sin(yaw);
    v = v + translational_acceleration_noise*delta_t;

    yaw = yaw + 0.5*rotational_acceleration_noise*delta_t*delta_t;
    yaw_rate = yaw_rate + rotational_acceleration_noise*delta_t;

    // write predicted sigma point into right column
    sigma_points(0,i) = p_x;
    sigma_points(1,i) = p_y;
    sigma_points(2,i) = v;
    sigma_points(3,i) = yaw;
    sigma_points(4,i) = yaw_rate;
  }
}

void UKF::PredictGaussian(const Eigen::MatrixXd& sigma_points){
  // create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i < (2 * n_aug_ + 1); ++i) {  // 2n+1 weights
    double weight = 0.5/(lambda_ + n_aug_);
    weights(i) = weight;
  }

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); ++i) {  // iterate over sigma points
    x_ = x_ + weights(i) * sigma_points.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = sigma_points.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose();
  }
}

void UKF::Prediction(double delta_t) {
  MatrixXd augmented_sigma_points = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  GenerateAugmentedSigmaPoints(augmented_sigma_points);
  PredictAugmentedSigmaPoints(augmented_sigma_points, delta_t);
  PredictGaussian(augmented_sigma_points);
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}
