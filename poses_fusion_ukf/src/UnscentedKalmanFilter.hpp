// UnscentedKalmanFilter.h
// Unscented Kalman Filter for poses fusion
// Created on: 2020.02.17
// Author: Chao Li
// Email: chao.li.arthur@gmail.com

#ifndef UKF_H
#define UKF_H

#include <ros/ros.h>
#include "Eigen/Dense"

#include "containers/data_container.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
 public:
  /**
   * Constructor
   */
  UKF(
      ///* State dimension
      int state_dimension,  //n_x

      ///* Sigma points dimension
      int sigma_points_dimension,

      ///* Augmented state dimension
      int augmented_dimension,  //n_aug

      ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      VectorXd vec_state,  //x

      ///* state covariance matrix
      MatrixXd mat_cov_state,  //P

      ///* predicted sigma points matrix
      MatrixXd mat_state_sigma_points_predict,  //xsig

      ///* Process noise standard deviation longitudinal acceleration in m/s^2
      double standard_devi_proc_noise_acceleration,

      ///* Process noise standard deviation yaw acceleration in rad/s^2
      double standard_devi_proc_noise_yaw,

      ///* Laser measurement noise standard deviation position1 in m
      double standard_devi_measure_noise_odom_ndt_px,

      ///* Laser measurement noise standard deviation position2 in m
      double standard_devi_measure_noise_odom_ndt_py,
    

      double standard_devi_measure_noise_odom_ndt_yaw_,
    
      double standard_devi_measure_noise_odom_ndt_yawd_,

      double standard_devi_measure_noise_odom_ndt_vx_,

      double standard_devi_measure_noise_odom_ndt_vy_,

      ///* Weights of sigma points
      VectorXd vec_sigma_points_weights,

      ///* Sigma point spreading parameter
      double sigma_point_spreading_param_lambda,

      ///* Lidar measurement noise covariance matrix
      MatrixXd mat_measure_noise_cov_odom_ndt,  //R_odom_ndt,

      ///* the current NIS - lidar
      double NIS_odom_ndt
      ) ;

  /**
   * Destructor
   */
  virtual ~UKF();

  double getVecState(int num);

  void stateLateInitializer(
      ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      VectorXd vec_state,  //x
      ///* Process noise standard deviation longitudinal acceleration in m/s^2
      double standard_devi_proc_noise_acceleration,

      ///* Process noise standard deviation yaw acceleration in rad/s^2
      double standard_devi_proc_noise_yaw,

      ///* Laser measurement noise standard deviation position1 in m
      double standard_devi_measure_noise_odom_ndt_px,

      ///* Laser measurement noise standard deviation position2 in m
      double standard_devi_measure_noise_odom_ndt_py,

      double standard_devi_measure_noise_odom_ndt_yaw_,
    
      double standard_devi_measure_noise_odom_ndt_yawd_,

      double standard_devi_measure_noise_odom_ndt_vx_,

      double standard_devi_measure_noise_odom_ndt_vy_,
       ///* Lidar measurement noise covariance matrix
      MatrixXd mat_measure_noise_cov_odom_ndt  //ndt_odom_ndt,
  );

  /**
   *   Generate sigma points:
   *  @param x : State vector.
   *  @param P : Covariance matrix.
   *  @param lambda: Sigma points spreading parameter.
   *  @param n_sig: Sigma points dimension.
   */
  MatrixXd generateSigmaPoints(VectorXd vec_state, MatrixXd mat_cov_state,
                               double sigma_point_spreading_param_lambda,
                               int sigma_points_dimension);

  /**
   * Predits sigma points.
   * @param Xsig : Sigma points to predict.
   * @param delta_t : Time between k and k+1 in s
   * @param n_x : State dimension.
   * @param n_sig : Sigma points dimension.
   * @param nu_am : Process noise standard deviation longitudinal acceleration in m/s^2
   * @param nu_yawdd : Process noise standard deviation yaw acceleration in rad/s^2
   */
  MatrixXd predictSigmaPoints(MatrixXd sigma_points_predict, double delta_t, int state_dimension, int sigma_points_dimension,
                              double standard_devi_proc_noise_acceleration, double standard_devi_proc_noise_yaw);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a lidar measurement
   * @param ego_state The measurement at k+1
   */
  void updateLidarNDT(ego_state_ts ego_state);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param ego_state The measurement at k+1
   */

    /**
   * Updates the state and the state covariance matrix using a lidar measurement
   * @param ego_state The measurement at k+1
   */
  void updateOdom(ego_state_ts ego_state);

 private:
  ///* State dimension
  int state_dimension_;  //n_x

  ///* Sigma points dimension
  int sigma_points_dimension_;

  ///* Augmented state dimension
  int augmented_dimension_;  //n_aug

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd vec_state_;  //x

  ///* state covariance matrix
  MatrixXd mat_cov_state_;  //P

  ///* predicted sigma points matrix
  MatrixXd mat_state_sigma_points_predict_;  //xsig

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double standard_devi_proc_noise_acceleration_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double standard_devi_proc_noise_yaw_;

  ///* Laser measurement noise standard deviation position1 in m
  double standard_devi_measure_noise_odom_ndt_px_;

  ///* Laser measurement noise standard deviation position2 in m
  double standard_devi_measure_noise_odom_ndt_py_;

  double standard_devi_measure_noise_odom_ndt_yaw_;
    
  double standard_devi_measure_noise_odom_ndt_yawd_;

  double standard_devi_measure_noise_odom_ndt_vx_;

  double standard_devi_measure_noise_odom_ndt_vy_;

  ///* Weights of sigma points
  VectorXd vec_sigma_points_weights_;

  ///* Sigma point spreading parameter
  double sigma_point_spreading_param_lambda_;

  ///* Lidar measurement noise covariance matrix
  MatrixXd mat_measure_noise_cov_odom_ndt_;  //R_odom_ndt,

  ///* the current NIS - lidar
  double NIS_odom_ndt_;

  std::default_random_engine gen;
};

#endif /* UKF_H */
