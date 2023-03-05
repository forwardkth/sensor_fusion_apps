// UnscentedKalmanFilter.cpp
// Unscented Kalman Filter for sensor fusion
// Created on: 2020.02.17
// Author: Chao Li
// Email: chao.li.arthur@gmail.com

#include <stdio.h>
#include "filters/UnscentedKalmanFilter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
/**
 * Initializes Unscented Kalman filter part 1
 */
UKF::UKF(

    ///* State dimension
    int state_dimension,  //n_x

    ///* Sigma points dimension
    int sigma_points_dimension,

    ///* Augmented state dimension
    int augmented_dimension,  //n_aug

    ///* state vector: [pos_x pos_y vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd vec_state, 

    ///* state covariance matrix
    MatrixXd mat_cov_state,  //P

    ///* predicted sigma points matrix
    MatrixXd mat_state_sigma_points_predict,  //mat_state_sigma_points_predict

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double standard_devi_proc_noise_acceleration,

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double standard_devi_proc_noise_yaw,

    ///* Lidar measurement noise standard deviation position1 in m
    double standard_devi_measure_noise_odom_ndt_px,

    ///* Lidar measurement noise standard deviation position2 in m
    double standard_devi_measure_noise_odom_ndt_py,
    
    double standard_devi_measure_noise_odom_ndt_yaw,
    
    double standard_devi_measure_noise_odom_ndt_yawd,

    double standard_devi_measure_noise_odom_ndt_vx,

    double standard_devi_measure_noise_odom_ndt_vy,
    ///* Weights of sigma points
    VectorXd vec_sigma_points_weights,

    ///* Sigma point spreading parameter
    double sigma_point_spreading_param_lambda,

    ///* Lidar measurement noise covariance matrix
    MatrixXd mat_measure_noise_cov_odom_ndt,  //R_odom_ndt,

    ///* the current NIS - lidar
    double NIS_odom_ndt):
      state_dimension_(state_dimension),
      sigma_points_dimension_(sigma_points_dimension),
      augmented_dimension_(augmented_dimension),
      vec_state_(vec_state),
      mat_cov_state_(mat_cov_state),
      mat_state_sigma_points_predict_(mat_state_sigma_points_predict),
      //time_us_(time_us),
      standard_devi_proc_noise_acceleration_(
          standard_devi_proc_noise_acceleration),
      standard_devi_proc_noise_yaw_(standard_devi_proc_noise_yaw),
      standard_devi_measure_noise_odom_ndt_px_(
          standard_devi_measure_noise_odom_ndt_px),
      standard_devi_measure_noise_odom_ndt_py_(
          standard_devi_measure_noise_odom_ndt_py),
      standard_devi_measure_noise_odom_ndt_yaw_(
          standard_devi_measure_noise_odom_ndt_yaw),
      standard_devi_measure_noise_odom_ndt_yawd_(
          standard_devi_measure_noise_odom_ndt_yawd),
      standard_devi_measure_noise_odom_ndt_vx_(
       standard_devi_measure_noise_odom_ndt_vx),
      standard_devi_measure_noise_odom_ndt_vy_(
      standard_devi_measure_noise_odom_ndt_vy),
      vec_sigma_points_weights_(vec_sigma_points_weights),
      sigma_point_spreading_param_lambda_(sigma_point_spreading_param_lambda),
      mat_measure_noise_cov_odom_ndt_(mat_measure_noise_cov_odom_ndt),
      NIS_odom_ndt_(NIS_odom_ndt) { }

UKF::~UKF() { }

double UKF::getVecState(int num)
{
  double state_result = this->vec_state_(num);
  return state_result;
}

void UKF::stateLateInitializer(
      ///* state vector: [pos_x pos_y vel_abs yaw_angle yaw_rate] in SI units and rad
      VectorXd vec_state,
      ///* Process noise standard deviation longitudinal acceleration in m/s^2
      double standard_devi_proc_noise_acceleration,

      ///* Process noise standard deviation yaw acceleration in rad/s^2
      double standard_devi_proc_noise_yaw,

      ///* Laser measurement noise standard deviation position1 in m
      double standard_devi_measure_noise_odom_ndt_px,

      ///* Laser measurement noise standard deviation position2 in m
      double standard_devi_measure_noise_odom_ndt_py,
      
      double standard_devi_measure_noise_odom_ndt_yaw,
    
      double standard_devi_measure_noise_odom_ndt_yawd,
      
      double standard_devi_measure_noise_odom_ndt_vx,

      double standard_devi_measure_noise_odom_ndt_vy,
       ///* Lidar measurement noise covariance matrix
      MatrixXd mat_measure_noise_cov_odom_ndt)
{
  vec_state_ = vec_state;
  standard_devi_proc_noise_acceleration_ = standard_devi_proc_noise_acceleration;
  standard_devi_proc_noise_yaw_ = standard_devi_proc_noise_yaw;
  standard_devi_measure_noise_odom_ndt_px_ = standard_devi_measure_noise_odom_ndt_px;
  standard_devi_measure_noise_odom_ndt_py_ = standard_devi_measure_noise_odom_ndt_py;
  standard_devi_measure_noise_odom_ndt_yaw_ = standard_devi_measure_noise_odom_ndt_yaw;
  standard_devi_measure_noise_odom_ndt_yawd_ = standard_devi_measure_noise_odom_ndt_yawd_;
  standard_devi_measure_noise_odom_ndt_vx_ = standard_devi_measure_noise_odom_ndt_vx;
  standard_devi_measure_noise_odom_ndt_vy_ = standard_devi_measure_noise_odom_ndt_vy;
  mat_measure_noise_cov_odom_ndt_ = mat_measure_noise_cov_odom_ndt; 
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::prediction(double delta_t)
{
  std::ios::sync_with_stdio(false);
#ifdef DEBUG
  printf("start prediction step\n");
#endif
  // 1. Generate sigma points.
  //create augmented mean vector
  // [x, y, velocity, yaw, yawrate, proc_noise_acc, proc_noise_yaw]
  VectorXd vec_state_aug_ = VectorXd(augmented_dimension_);
  vec_state_aug_.head(5) = vec_state_; // [x, y, velocity, yaw, yawrate]
  //std::cout <<"input to prediction yaw "<< vec_state_(3) <<std::endl;
  vec_state_aug_(5) = 0;
  vec_state_aug_(6) = 0;

  vec_state_aug_(5) = 0;


  //create augmented state covariances
  MatrixXd mat_cov_state_aug_ = MatrixXd(augmented_dimension_,
                                         augmented_dimension_);
  mat_cov_state_aug_.fill(0.0);
  mat_cov_state_aug_.topLeftCorner(state_dimension_, state_dimension_) =
      mat_cov_state_;
  mat_cov_state_aug_(5, 5) = standard_devi_proc_noise_acceleration_
      * standard_devi_proc_noise_acceleration_;
  mat_cov_state_aug_(6, 6) = standard_devi_proc_noise_yaw_
      * standard_devi_proc_noise_yaw_;

  //1. Create sigma points of augmented states.
#ifdef DEBUG
  printf("create augmented sigma points\n");
#endif

  MatrixXd mat_state_sigma_points_predict_aug_ = generateSigmaPoints(
      vec_state_aug_, mat_cov_state_aug_, sigma_point_spreading_param_lambda_,
      sigma_points_dimension_);
  // 2. Predict Sigma Points.
  mat_state_sigma_points_predict_ = predictSigmaPoints(
      mat_state_sigma_points_predict_aug_, delta_t, state_dimension_,
      sigma_points_dimension_, standard_devi_proc_noise_acceleration_,
      standard_devi_proc_noise_yaw_);
  // 3. Predict Mean and Covariance
  //predicted state mean
  vec_state_ = mat_state_sigma_points_predict_ * vec_sigma_points_weights_;

  //predicted state covariance matrix
  mat_cov_state_.fill(0.0);
  for (int i = 0; i < sigma_points_dimension_; i++)
  {  //iterate over sigma points

    // state difference
    VectorXd vec_state_diff = mat_state_sigma_points_predict_.col(i)
        - vec_state_;
    //angle normalization
    if (vec_state_diff(3) > M_PI)
    {
      vec_state_diff(3) -= 2. * M_PI;
    }
    else if (vec_state_diff(3) < -M_PI)
    {
      vec_state_diff(3) += 2. * M_PI;
    }
    mat_cov_state_ = mat_cov_state_
        + vec_sigma_points_weights_(i) * vec_state_diff
            * vec_state_diff.transpose();
  }
#ifdef DEBUG
  printf("++++finish prediction once++++\n");
#endif
}

/****************************************************************************************
 * Updates the state and the state covariance matrix using a lidar based NDT measurement.
 * 
 ****************************************************************************************/
void UKF::updateLidarNDT(ego_state_ts ego_state)
{
  std::ios::sync_with_stdio(false);

#ifdef DEBUG
  printf("start update lidar-------------------------\n");
#endif
  // 1. Predit measurement
  int n_z_ = 2;

  // define the Zsig_ points equal to mat_state_sigma_points_predict_ variable.
  MatrixXd Zsig_ = mat_state_sigma_points_predict_.block(
      0, 0, n_z_, sigma_points_dimension_);

  //mean predicted measurement　［x, y］
  VectorXd z_pred_ = VectorXd(n_z_);
  z_pred_.fill(0.0);
  for (int i = 0; i < sigma_points_dimension_; i++)
  {
    z_pred_ = z_pred_ + vec_sigma_points_weights_(i) * Zsig_.col(i);
  }
#ifdef DEBUG
  printf("calculate measurement covariance matrix S\n");
#endif
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);
  for (int i = 0; i < sigma_points_dimension_; i++)
  {  //2n+1 simga points
     //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    S = S + vec_sigma_points_weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + mat_measure_noise_cov_odom_ndt_;

  // 2. Update state
  // Incoming radar measurement
#ifdef DEBUG
  printf("update measurement with sensor data/n");
#endif
  VectorXd z = VectorXd(n_z_);
  z(0) = ego_state.mean_x;
  z(1) = ego_state.mean_y;

  //create matrix for cross correlation Tc
#ifdef DEBUG
  printf("calculation the cross correlation Tc lidar\n");
#endif
  MatrixXd Tc = MatrixXd(state_dimension_, n_z_);

  Tc.fill(0.0);
#ifdef DEBUG
  printf("calculation tc with x error and z error\n");
#endif
  for (int i = 0; i < sigma_points_dimension_; i++)
  {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    // state difference
    VectorXd vec_state_diff = mat_state_sigma_points_predict_.col(i)
        - vec_state_;

    Tc = Tc + vec_sigma_points_weights_(i) * vec_state_diff * z_diff.transpose();
  }

  //Kalman gain K;k=tc*s^(-1)
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred_;

  //update state mean and covariance matrix
#ifdef DEBUG
  printf("update lidar state mean covariance matrix\n");
#endif
  vec_state_ = vec_state_ + K * z_diff;
  mat_cov_state_ = mat_cov_state_ - K * S * K.transpose();
#ifdef DEBUG
  printf("calculation the NIS_odom_ndt\n");
#endif
  NIS_odom_ndt_ = z_diff.transpose() * S.inverse() * z_diff;
#ifdef DEBUG
  printf("finish  NIS lidar calculation\n");
#endif
}
/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {ego_state} ego_state
 */
void UKF::updateOdom(ego_state_ts ego_state)
{
  UKF::updateLidarNDT(ego_state);
}

/**
 * Predits sigma points.
 */
MatrixXd UKF::predictSigmaPoints(MatrixXd mat_state_sigma_points_predict,
                                 double delta_t, int n_x, int n_sig,
                                 double nu_am, double nu_yawdd)
{
  MatrixXd mat_state_sigma_points_predict_pred = MatrixXd(n_x, n_sig);
  //predict sigma points
  for (int i = 0; i < n_sig; i++)
  {
    //extract values for better readability
    double mat_cov_state_x = mat_state_sigma_points_predict(0, i);
    double mat_cov_state_y = mat_state_sigma_points_predict(1, i);
    double v = mat_state_sigma_points_predict(2, i);
    double yaw = mat_state_sigma_points_predict(3, i);
    double yawd = mat_state_sigma_points_predict(4, i);
    double nu_a = mat_state_sigma_points_predict(5, i);
    double nu_yawdd = mat_state_sigma_points_predict(6, i);

    std::normal_distribution<double> dist_x(0, standard_devi_measure_noise_odom_ndt_px_);
    std::normal_distribution<double> dist_y(0, standard_devi_measure_noise_odom_ndt_py_);
    std::normal_distribution<double> dist_theta(0, standard_devi_measure_noise_odom_ndt_yaw_); 
    std::normal_distribution<double> dist_velx(0, standard_devi_measure_noise_odom_ndt_vx_);
	  std::normal_distribution<double> dist_vely(0, standard_devi_measure_noise_odom_ndt_vy_);	
    std::normal_distribution<double> dist_thetad(0, standard_devi_measure_noise_odom_ndt_yawd_);

    //predicted state values
    double x_predict;
    double y_predict;
    double yaw_p;
    //avoid division by zero
    // predict with the bicycle model
    if (fabs(yawd) > 0.000001)
    {
      x_predict = mat_cov_state_x
          + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      y_predict = mat_cov_state_y
          + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
      
      yaw_p = yaw + yawd * delta_t;
    }
    else
    {
      x_predict = mat_cov_state_x + v * delta_t * cos(yaw);
      y_predict = mat_cov_state_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yawd_p = yawd;

    //add noise
    x_predict = x_predict + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    y_predict = y_predict + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;
    
    //write predicted sigma point into right column
    mat_state_sigma_points_predict_pred(0, i) = x_predict;
    mat_state_sigma_points_predict_pred(1, i) = y_predict;
    mat_state_sigma_points_predict_pred(2, i) = v_p;
    if(yaw_p <= -3.14)
    {
      yaw_p = yaw_p + 2.0 * 3.1415926;
    }
    mat_state_sigma_points_predict_pred(3, i) = yaw_p;
    mat_state_sigma_points_predict_pred(4, i) = yawd_p;

  }

  return mat_state_sigma_points_predict_pred;
}
/**
 *   Generate sigma points:
 */
MatrixXd UKF::generateSigmaPoints(VectorXd x, MatrixXd P, double lambda,
                                  int n_sig)
{
  int n = x.size();
  //create sigma point matrix
  MatrixXd mat_state_sigma_points_predict = MatrixXd(n, n_sig);

  //calculate square root of P
  MatrixXd A = P.llt().matrixL();

  mat_state_sigma_points_predict.col(0) = x;

  double lambda_plue_n_vec_state_sqrt = sqrt(lambda + n);
  for (int i = 0; i < n; i++)
  {
    mat_state_sigma_points_predict.col(i + 1) = x
        + lambda_plue_n_vec_state_sqrt * A.col(i);
    mat_state_sigma_points_predict.col(i + 1 + n) = x
        - lambda_plue_n_vec_state_sqrt * A.col(i);
  }
  return mat_state_sigma_points_predict;
}
