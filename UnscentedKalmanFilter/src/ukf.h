#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
 * Prediction Predicts sigma points, the state, and the state covariance
 * matrix
 * @param delta_t Time between k and k+1 in s
 */
  void Prediction(double delta_t);

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);


  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Generates a set of sigma points
   */
  void GenerateAugmentedSigmaPoints();

  /**
   * Processes a given set of sigma points through the motion model
   * @param delta_t time between two consecutive time samples
   */
  void PredictAugmentedSigmaPoints(double delta_t);

  /**
   * Calculates a-priori distribution
   */
  void PredictGaussian();

  /**
   * Convert predicted Gaussian from state space into measurement space
   * @param measurement_sigma_points: matrix storing the sigma points in the measurement space
   * @param predicted_measurements: vector containing predicted Gaussian in the measurement space
   * @param S: Innovation covariance matrix
   */
  void PredictRadarMeasurement(Eigen::MatrixXd& measurement_sigma_points, Eigen::VectorXd& predicted_measurements, Eigen::MatrixXd& S);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // matrix storing the sigma points in the state space
  Eigen::MatrixXd state_sigma_points_;

  // Timestamp when the state was last updated based on sensor measurements
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // radar measurement dimension
  int radar_n_z;

  // lidar measurement dimension
  int lidar_n_z;

  // Sigma point spreading parameter
  double lambda_;
};

#endif  // UKF_H
