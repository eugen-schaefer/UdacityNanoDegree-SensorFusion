#include <cfloat>
#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

inline bool isEqual(double x, double y)
{
  return std::abs(x - y) <= DBL_EPSILON;
}

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

  // set measurement dimension, radar can measure r, phi, and r_dot
  radar_n_z = 3;

  // set measurement dimension, lidar can measure pos_x and pos_y
  lidar_n_z = 2;

  // define spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = Eigen::VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(n_x_, n_x_);

  state_sigma_points_ = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);

  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i < (2 * n_aug_ + 1); ++i) {  // 2n+1 weights
    weights_(i) = 0.5/(lambda_ + n_aug_);
  }

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

  is_initialized_ = false;

  time_us_ = 0;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */
}

UKF::~UKF() {}

// Transform Gaussian (mean and covariance) into sigma points
void UKF::GenerateAugmentedSigmaPoints(){
  // create augmented state mean vector
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  // create augmented state covariance matrix
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  // take a square root of the augmented covariance matrix
  MatrixXd sqrt_P_aug = P_aug.llt().matrixL();

  // create (2n + 1) augmented sigma points
  state_sigma_points_.col(0)  = x_aug;
  for (int idx = 0; idx < n_aug_; ++idx) {
    state_sigma_points_.col(idx+1)        = x_aug + sqrt(lambda_+n_aug_) * sqrt_P_aug.col(idx);
    state_sigma_points_.col(idx+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * sqrt_P_aug.col(idx);
  }
}

// Process sigma points through the process model
void UKF::PredictAugmentedSigmaPoints(double delta_t){
  for (int idx = 0; idx< 2*n_aug_+1; ++idx){
    // extract values from the sigma point matrix for better readability
    double long_pos_sp = state_sigma_points_(0, idx);
    double lat_pos_sp = state_sigma_points_(1, idx);
    double abs_vel_sp = state_sigma_points_(2, idx);
    double yaw_sp = state_sigma_points_(3, idx);
    double yaw_rate_sp = state_sigma_points_(4, idx);
    double translational_acceleration_noise_sp = state_sigma_points_(5, idx);
    double rotational_acceleration_noise_sp = state_sigma_points_(6, idx);

    // propagate sigma points through the process model avoiding division by zero
    if (fabs(yaw_rate_sp) > 0.001) {
      long_pos_sp = long_pos_sp + abs_vel_sp/yaw_rate_sp * ( sin (yaw_sp + yaw_rate_sp*delta_t) - sin(yaw_sp));
      lat_pos_sp = lat_pos_sp + abs_vel_sp/yaw_rate_sp * ( cos(yaw_sp) - cos(yaw_sp+yaw_rate_sp*delta_t) );
    } else {
      long_pos_sp = long_pos_sp + abs_vel_sp*delta_t*cos(yaw_sp);
      lat_pos_sp = lat_pos_sp + abs_vel_sp*delta_t*sin(yaw_sp);
    }
    yaw_sp = yaw_sp + yaw_rate_sp*delta_t;

    // add noise
    long_pos_sp = long_pos_sp + 0.5*translational_acceleration_noise_sp*delta_t*delta_t * cos(yaw_sp);
    lat_pos_sp = lat_pos_sp + 0.5*translational_acceleration_noise_sp*delta_t*delta_t * sin(yaw_sp);
    abs_vel_sp = abs_vel_sp + translational_acceleration_noise_sp*delta_t;
    yaw_sp = yaw_sp + 0.5*rotational_acceleration_noise_sp*delta_t*delta_t;
    // angle normalization
    while (yaw_sp > M_PI) yaw_sp-=2.*M_PI;
    while (yaw_sp <-M_PI) yaw_sp+=2.*M_PI;
    yaw_rate_sp = yaw_rate_sp + rotational_acceleration_noise_sp*delta_t;

    // write back predicted sigma points into the sigma point matrix
    state_sigma_points_(0, idx) = long_pos_sp;
    state_sigma_points_(1, idx) = lat_pos_sp;
    state_sigma_points_(2, idx) = abs_vel_sp;
    state_sigma_points_(3, idx) = yaw_sp;
    state_sigma_points_(4, idx) = yaw_rate_sp;
  }
}

// Transform back sigma points into Gaussian (mean and covariance)
void UKF::PredictGaussian(){

  // predicted state mean
  x_.fill(0.0);
  for (int idx = 0; idx < (2 * n_aug_ + 1); ++idx) {
    x_ = x_ + weights_(idx) * state_sigma_points_.col(idx).segment(0, n_x_);
  }

  // In case we get unrealistic values due to numerical issues,
  // reset entire estimation and restart initialization
  for (int idx = 0; idx < n_x_; ++idx){
    if (std::isinf(x_(idx)) || std::isnan(x_(idx)) || std::abs(x_(idx)) > 1000 ){
      x_.fill(0.0);
      P_.fill(0.1);
      is_initialized_ = false;
      std::cout << "Restart initialization due to numerical issues in the prediction step" << std::endl;
      return;
    }
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int idx = 0; idx < (2 * n_aug_ + 1); ++idx) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = state_sigma_points_.col(idx).segment(0, n_x_) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P_ = P_ + weights_(idx) * x_diff * x_diff.transpose();
  }
}

void UKF::Prediction(double delta_t) {
  GenerateAugmentedSigmaPoints();
  PredictAugmentedSigmaPoints(delta_t);
  PredictGaussian();
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // Time difference in seconds between the last estimate (posterior) and the timestamp from fresh measurement
  // used to predict the posterior to the time where measurement has been captured and hence to align the
  // posterior with newly arrived measurement in terms of time.
  double dt = (static_cast<double>(meas_package.timestamp_) - static_cast<double>(time_us_))/1000000.0;
  if (is_initialized_ && (dt > 0)){
    Prediction(dt);
  }

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR){
    if(!is_initialized_){
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);

      double init_long_pos = rho * cos(phi);
      double init_lat_pos  = rho * sin(phi);
      double init_long_vel = rho_dot * cos(phi);
      double init_lat_vel  = rho_dot * sin(phi);
      double init_abs_vel = sqrt(init_long_pos*init_long_pos + init_lat_pos*init_lat_pos);
      double init_yaw = atan2(init_lat_vel, init_long_vel);
      double init_yawrate = 0.0;

      x_ << init_long_pos, init_lat_pos, init_abs_vel, init_yaw, init_yawrate;

      double init_sigma_x{std_radr_ * std_radr_};
      double init_sigma_y{std_radr_ * std_radr_};
      double init_sigma_v{std_radrd_ * std_radrd_};
      double init_sigma_yaw{std_radphi_*std_radphi_};
      double init_sigma_yawrate{std_radrd_*std_radrd_};
      P_.fill(0.0);
      P_.diagonal() << init_sigma_x, init_sigma_y, init_sigma_v, init_sigma_yaw, init_sigma_yawrate;

      GenerateAugmentedSigmaPoints();
      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;
    }

    UpdateRadar(meas_package);
    time_us_ = meas_package.timestamp_;
  }
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
  Eigen::VectorXd predicted_measurements = Eigen::VectorXd(radar_n_z);
  Eigen::MatrixXd S = Eigen::MatrixXd(radar_n_z, radar_n_z);
  Eigen::MatrixXd measurement_sigma_points = Eigen::MatrixXd(radar_n_z, 2 * n_aug_ + 1);

  PredictRadarMeasurement(measurement_sigma_points, predicted_measurements, S);

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, radar_n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = measurement_sigma_points.col(i) - predicted_measurements;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    Eigen::VectorXd predicted_x = VectorXd(n_x_);
    predicted_x = state_sigma_points_.col(i).segment(0, 5);
    VectorXd x_diff = predicted_x - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - predicted_measurements;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();


  // In case we get unrealistic values due to numerical issues,
  // reset entire estimation and restart initialization
  for (int idx = 0; idx < n_x_; ++idx){
    if (std::isinf(x_(idx)) || std::isnan(x_(idx)) || std::abs(x_(idx)) > 1000 ){
      x_.fill(0.0);
      P_.fill(0.1);
      is_initialized_ = false;
      std::cout << "Restart initialization due to numerical issues in state mean after measurement update" << std::endl;
      return;
    }
    if (P_(idx, idx) > 1000 ){
      x_.fill(0.0);
      P_.fill(0.1);
      is_initialized_ = false;
      std::cout << "Restart initialization due to numerical issues in state covariance after measurement update" << std::endl;
      return;
    }
  }
}


void UKF::PredictRadarMeasurement(Eigen::MatrixXd& measurement_sigma_points, Eigen::VectorXd& predicted_measurements, Eigen::MatrixXd& S){

  // transform (2n+1) predicted sigma points from state space into the measurement space
  for (int idx = 0; idx < (2 * n_aug_ + 1); ++idx) {
    // extract values for better readability
    double long_pos_sp = state_sigma_points_(0, idx);
    double lat_pos_sp = state_sigma_points_(1, idx);
    double abs_vel_sp  = state_sigma_points_(2, idx);
    double yaw_sp = state_sigma_points_(3, idx);
    double long_vel_sp = cos(yaw_sp)*abs_vel_sp;
    double lat_vel_sp = sin(yaw_sp)*abs_vel_sp;

    // create measurement model
    measurement_sigma_points(0, idx) = sqrt(long_pos_sp*long_pos_sp + lat_pos_sp*lat_pos_sp);
    if (isEqual(long_pos_sp, 0.0) || isEqual(lat_pos_sp, 0.0)){
      measurement_sigma_points(1, idx) = 0.01;
    } else{
      measurement_sigma_points(1, idx) = atan2(lat_pos_sp, long_pos_sp);
    }
    if (isEqual(long_pos_sp*long_pos_sp + lat_pos_sp*lat_pos_sp, 0.0)){
      measurement_sigma_points(2, idx) = 0.01;
    }else{
      measurement_sigma_points(2, idx) = (long_pos_sp*long_vel_sp + lat_pos_sp*lat_vel_sp) / sqrt(long_pos_sp*long_pos_sp + lat_pos_sp*lat_pos_sp); // r_dot
    }
  }

  // predicted mean z(k+1) in the measurement space
  predicted_measurements.fill(0.0);
  for (int i=0; i < (2 * n_aug_ + 1); ++i) {
    predicted_measurements = predicted_measurements + weights_(i) * measurement_sigma_points.col(i);
  }

  // predicted covariance in the measurement space --> innovation covariance matrix S(k+1)
  S.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); ++i) {
    // residual
    VectorXd z_diff = measurement_sigma_points.col(i) - predicted_measurements;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise to innovation covariance matrix
  Eigen::MatrixXd R = Eigen::MatrixXd(radar_n_z, radar_n_z);
  R <<  std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0,std_radrd_*std_radrd_;
  S = S + R;
}
