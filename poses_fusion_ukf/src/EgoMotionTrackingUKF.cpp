// EgoMotionTrackingUKF.cpp
// Unscented Kalman Filter for sensor fusion node
// Created on: 2020.02.17
// Author: Chao Li
// Email: chao.li.arthur@gmail.com

#include <stdio.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "sensor_fusion/EgoMotionTrackingUKF.h"
#include "filters/UnscentedKalmanFilter.h"

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;

EgoMotionTrackingUKF::EgoMotionTrackingUKF(ros::NodeHandle nh, ros::NodeHandle priv_nh)
    :nh_(nh), priv_nh_(priv_nh)
{
  // init the velocity and theta global variables to zero. 
  velocity_global_ = 0.0;
  theta_global_ = 0.0;

  // init frame and yaw rate covariance from ros parameters.
  
  ros::param::param<std::string>("~frame", frame_, "map");
  ros::param::param<double>("~covariance_yawrate", covariance_yawrate_);
  // ros::param::get("~prediction_freq", prediction_freq_);

  // use message filter to enable the flexibility for synchronization
  ndt_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh_, "ndt", 1);
  ndt_sub_ -> registerCallback(boost::bind(&EgoMotionTrackingUKF::ndtCallback, this, _1));

  odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry> (nh_, "odom", 1);
  odom_sub_ -> registerCallback(boost::bind(&EgoMotionTrackingUKF::odomCallback, this, _1));

  // timer_ = nh_.createTimer(ros::Duration(1/prediction_freq_), &EgoMotionTrackingUKF::timerCallback,this);
    
  // track_synchronizer_ = new message_filters::Synchronizer<SyncTrackPolicyT>(
  //           SyncTrackPolicyT(10), *ndt_sub_, *odom_sub_);
  // track_synchronizer_ -> registerCallback(
  //           boost::bind(&EgoMotionTrackingUKF::timeSyncCallback, this, _1, _2));

  fused_state_pub_ = nh_.advertise<nav_msgs::Odometry>("output", 1,this);
 
  ///*----------UKF initialization start part 1----------*///

  // initialize the init flag with "false"
  init_done_ = false;;
  gps_update_lock_ = true;
  ndt_update_lock_ = true;

  // use odom to update
  // priv_nh_.param::get("~use_odom", use_odom_);
  use_odom_ = true;
  // use ndt to update
  // priv_nh_.param::get("~use_odom_ndt_ndt_", use_odom_ndt_ndt_);
  use_ndt_ = true;

  ///* State dimension
  state_dimension_ = 5;  //n_x

  ///* Augmented state dimension
  //n_aug 
  augmented_dimension_ = state_dimension_ + 2;

  ///* Sigma points dimension
  //number of the sigma points selection, 2n+1
  sigma_points_dimension_ = 2 * augmented_dimension_ + 1;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  vec_state_ = VectorXd(state_dimension_);  //x

  ///* state covariance matrix
  mat_cov_state_ = MatrixXd(state_dimension_, state_dimension_);  //P

  mat_cov_state_ << 1, 0, 0, 0, 0,
                    0, 1, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;

  ///* predicted sigma points matrix
  mat_state_sigma_points_predict_ = MatrixXd(state_dimension_,
                                                     sigma_points_dimension_);  //xsig

  ///* time when the state is true, in us
  //long long time_us = 0.0;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2 1.6
  standard_devi_proc_noise_acceleration_ = 1.6;

  ///* Process noise standard deviation yaw acceleration in rad/s^2 0.6
  standard_devi_proc_noise_yaw_ = 0.1;

  ///* Lidar measurement noise standard deviation position1 in m
  standard_devi_measure_noise_odom_ndt_px_ = 0.15;

  ///* Lidar measurement noise standard deviation position2 in m
  standard_devi_measure_noise_odom_ndt_py_ = 0.15;
  
  standard_devi_measure_noise_odom_ndt_yaw_ = 0.0;

  standard_devi_measure_noise_odom_ndt_yawd_ = 0.0;

  standard_devi_measure_noise_odom_ndt_vx_ = 0.0;

  standard_devi_measure_noise_odom_ndt_vy_ = 0.0;

  ///* Weights of sigma points
  vec_sigma_points_weights_ = VectorXd(sigma_points_dimension_);

  ///* Sigma point spreading parameter Lambda (the distance from sigma points to x)
  sigma_point_spreading_param_lambda_ = 3 - state_dimension_; //experience formula

  ///* Sigma points weights
  vec_sigma_points_weights_(0) = sigma_point_spreading_param_lambda_
      / (sigma_point_spreading_param_lambda_ + augmented_dimension_);
  for (int i = 1; i < sigma_points_dimension_; i++)
  {  //2n+1 weights
    vec_sigma_points_weights_(i) = 0.5
        / (sigma_point_spreading_param_lambda_ + augmented_dimension_);
  }

  ///* Lidar measurement noise covariance matrix
  mat_measure_noise_cov_odom_ndt_ = MatrixXd(2, 2);  //R_odom_ndt,
  mat_measure_noise_cov_odom_ndt_
      << standard_devi_measure_noise_odom_ndt_px_
          * standard_devi_measure_noise_odom_ndt_px_, 0, 0, standard_devi_measure_noise_odom_ndt_py_
      * standard_devi_measure_noise_odom_ndt_py_;

  ///* the current NIS - lidar
  NIS_odom_ndt_ = 0.0;


  ukf_ = new UKF(///* initially set to false, set to true in first call of ProcessMeasurement

      ///* State dimension
      state_dimension_,  //n_x

      ///* Sigma points dimension
      sigma_points_dimension_,

      ///* Augmented state dimension
      augmented_dimension_,  //n_aug

      ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      vec_state_,  //x

      ///* state covariance matrix
      mat_cov_state_,  //P

      ///* predicted sigma points matrix
      mat_state_sigma_points_predict_,  //xsig

      ///* time when the state is true, in us
      //time_us,

      ///* Process noise standard deviation longitudinal acceleration in m/s^2
      standard_devi_proc_noise_acceleration_,

      ///* Process noise standard deviation yaw acceleration in rad/s^2
      standard_devi_proc_noise_yaw_,

      ///* Laser measurement noise standard deviation position1 in m
      standard_devi_measure_noise_odom_ndt_px_,

      ///* Laser measurement noise standard deviation position2 in m
      standard_devi_measure_noise_odom_ndt_py_,

      standard_devi_measure_noise_odom_ndt_yaw_,
    
      standard_devi_measure_noise_odom_ndt_yawd_,

      standard_devi_measure_noise_odom_ndt_vx_,

      standard_devi_measure_noise_odom_ndt_vy_,

      ///* Weights of sigma points
      vec_sigma_points_weights_,

      ///* Sigma point spreading parameter
      sigma_point_spreading_param_lambda_,

      ///* Lidar measurement noise covariance matrix
      mat_measure_noise_cov_odom_ndt_,  //R_odom_ndt,

      ///* the current NIS - lidar
      NIS_odom_ndt_
  );
}

EgoMotionTrackingUKF::~EgoMotionTrackingUKF(){}

//odometry callback function
void EgoMotionTrackingUKF::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_state)
{
  std::ios::sync_with_stdio(false);

  static ros::Time previous_time = odom_state->header.stamp;
	ros::Time current_time = odom_state->header.stamp;
  
	//ros::Time gps_current_time = odom_state->header.stamp;
	gps_ego_current_state_.mean_x = odom_state->pose.pose.position.x;
	gps_ego_current_state_.mean_y = odom_state->pose.pose.position.y;
	gps_ego_current_state_.mean_z = odom_state->pose.pose.position.z;
	geometry_msgs::Quaternion qg = odom_state->pose.pose.orientation;
  tf::Quaternion q(qg.x, qg.y, qg.z, qg.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  gps_ego_current_state_.velocity = odom_state->twist.twist.linear.x;
  gps_ego_current_state_.yawRate = odom_state->twist.twist.angular.z;
  if(gps_ego_current_state_.velocity <= 0.3 && gps_ego_current_state_.velocity >= -0.3)
  {
    gps_ego_current_state_.yawRate = 0.0;
  }
  gps_ego_current_state_.mean_yaw = yaw; //experimental
  gps_ego_current_state_.mean_pitch = pitch; //experimental
  gps_ego_current_state_.sigmaX = odom_state->pose.covariance[0];
  gps_ego_current_state_.sigmaY = odom_state->pose.covariance[7];
  gps_ego_current_state_.sigmaYaw = odom_state->pose.covariance[35];
  gps_ego_current_state_.sigmaVelX = 0.01; //odom_state->twist.covariance[0];
  gps_ego_current_state_.sigmaVelY = 0.01; //odom_state->twist.covariance[7];
  gps_ego_current_state_.sigmaYawrate = covariance_yawrate_; //0.001; //odom->twist.covariance[35]; //missing covarience in odom msg from oxts
  gps_ego_current_state_.delta_t = (current_time - previous_time).toSec();
  
  printf("------------------------------------------------------\n");
  printf("ndt x: %f\n", ndt_ego_current_state_.mean_x);
  printf("gps x: %f\n", gps_ego_current_state_.mean_x);
  printf("ndt y: %f\n", ndt_ego_current_state_.mean_y);
  printf("gps y: %f\n", gps_ego_current_state_.mean_y);
  printf("ndt yaw: %f\n", ndt_ego_current_state_.mean_yaw);
  printf("gps yaw: %f\n", gps_ego_current_state_.mean_yaw);
  printf("ndt yawrate: %f\n", ndt_ego_current_state_.yawRate);
  printf("gps yawrate: %f\n", gps_ego_current_state_.yawRate);
  printf("ndt speed V: %f\n", ndt_ego_current_state_.velocity);
  printf("gps speed V: %f\n", gps_ego_current_state_.velocity);
  printf("Delta t: %f\n", gps_ego_current_state_.delta_t);

  previous_time = current_time;

  if (!gps_update_lock_) 
      return;
  
  // printf("------------------------------------------------------\n");
  // printf("gps std dev noise px: %f\n", gps_ego_current_state_.sigmaX);
  // printf("gps std dev noise py: %f\n", gps_ego_current_state_.sigmaY);
  // printf("gps std dev noise yaw: %f\n", gps_ego_current_state_.sigmaYaw);
  // printf("gps std dev noise yawd: %f\n", gps_ego_current_state_.sigmaYawrate);
  // printf("gps std dev noise vx: %f\n", gps_ego_current_state_.sigmaVelX);
  // printf("gps std dev noise vy: %f\n", gps_ego_current_state_.sigmaVelY);

  if(!init_done_) //Initialize Filter
	{
    if(gps_ego_current_state_.velocity > 0.3 || gps_ego_current_state_.velocity < - 0.3)
    {
      //[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      vec_state_ << gps_ego_current_state_.mean_x, gps_ego_current_state_.mean_y, 
      gps_ego_current_state_.velocity, gps_ego_current_state_.mean_yaw, gps_ego_current_state_.yawRate; 

      standard_devi_measure_noise_odom_ndt_px_ = 0.1; //gps_ego_current_state_.sigmaX * 100.0 * 5.0;//gps_ego_current_state_.sigmaX; 0.000256 ndt 0.02
      standard_devi_measure_noise_odom_ndt_py_ = 0.1; //gps_ego_current_state_.sigmaY * 100.0 * 5.0;// gps_ego_current_state_.sigmaY; 0.000323 ndt 0.02
      standard_devi_measure_noise_odom_ndt_yaw_ = 0.0006;  //ndt_ego_current_state_.sigmaYaw; 0.00012// gps_ego_current_state_.sigmaYaw; 0.000025
      standard_devi_measure_noise_odom_ndt_yawd_ = 0.01; //gps_ego_current_state_.sigmaYawrate; 0
      standard_devi_measure_noise_odom_ndt_vx_ = 0.01;; //gps_ego_current_state_.sigmaVelX; 0.01
      standard_devi_measure_noise_odom_ndt_vy_ = 0.01; //gps_ego_current_state_.sigmaVelY; 0.01

    
      ///* Lidar measurement noise covariance matrix
      mat_measure_noise_cov_odom_ndt_(0, 0) = standard_devi_measure_noise_odom_ndt_px_ * standard_devi_measure_noise_odom_ndt_px_;
      mat_measure_noise_cov_odom_ndt_(0, 1) = 0;
      mat_measure_noise_cov_odom_ndt_(1, 0) = 0;
      mat_measure_noise_cov_odom_ndt_(1, 1) = standard_devi_measure_noise_odom_ndt_py_ * standard_devi_measure_noise_odom_ndt_py_;

		  ukf_-> stateLateInitializer(     
      ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        vec_state_,
      ///* Process noise standard deviation longitudinal acceleration in m/s^2
        standard_devi_proc_noise_acceleration_,

      ///* Process noise standard deviation yaw acceleration in rad/s^2
        standard_devi_proc_noise_yaw_,

      ///* Laser measurement noise standard deviation position1 in m
        standard_devi_measure_noise_odom_ndt_px_,

      ///* Laser measurement noise standard deviation position2 in m
        standard_devi_measure_noise_odom_ndt_py_,

        standard_devi_measure_noise_odom_ndt_yaw_,

        standard_devi_measure_noise_odom_ndt_yawd_,

        standard_devi_measure_noise_odom_ndt_vx_,

        standard_devi_measure_noise_odom_ndt_vy_,
       ///* Lidar measurement noise covariance matrix
        mat_measure_noise_cov_odom_ndt_);

		  init_done_= true;
     }
	}
  else if(init_done_)
	{
    if(gps_ego_current_state_.velocity > 0.3 || gps_ego_current_state_.velocity < - 0.3)
    {
      ukf_ -> prediction(gps_ego_current_state_.delta_t);
      if(use_odom_ == true)
      {
        ukf_ -> updateOdom(gps_ego_current_state_);
        double x_out_gps = ukf_-> getVecState(0);
        double y_out_gps = ukf_-> getVecState(1);
        double yaw_out_gps = ukf_-> getVecState(3);
        printf("UKF filtered yaw updated by GPS: %f\n", yaw_out_gps);
        visualize_odom(yaw_out_gps);        
      }
    }
	}
}

void EgoMotionTrackingUKF::ndtCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ndt_state)
{
  std::ios::sync_with_stdio(false);

  static ros::Time previous_time = ndt_state->header.stamp;
	ros::Time current_time = ndt_state->header.stamp;

	ndt_ego_current_state_.mean_x = ndt_state->pose.pose.position.x;
	ndt_ego_current_state_.mean_y = ndt_state->pose.pose.position.y;
	ndt_ego_current_state_.mean_z = ndt_state->pose.pose.position.z;
	geometry_msgs::Quaternion ndt_qg = ndt_state->pose.pose.orientation;
	tf::Quaternion ndt_q(ndt_qg.x, ndt_qg.y, ndt_qg.z, ndt_qg.w);
	tf::Matrix3x3 ndt_m(ndt_q);
	double ndt_roll, ndt_pitch, ndt_yaw;
	ndt_m.getRPY(ndt_roll, ndt_pitch, ndt_yaw);


  ndt_ego_current_state_.mean_yaw = ndt_yaw; //experimental
	ndt_ego_current_state_.mean_pitch = ndt_pitch; //experimental
	ndt_ego_current_state_.sigmaX = ndt_state->pose.covariance[0];
	ndt_ego_current_state_.sigmaY = ndt_state->pose.covariance[7];
	ndt_ego_current_state_.sigmaYaw = ndt_state->pose.covariance[35];
	ndt_ego_current_state_.sigmaVelX = 0.01; //just like gps state
	ndt_ego_current_state_.sigmaVelY = 0.01; //just like gps state
	ndt_ego_current_state_.sigmaYawrate = covariance_yawrate_; //just like gps state
	ndt_ego_current_state_.delta_t = (current_time - previous_time).toSec();

  // ndt_ego_current_state_.velocity = gps_ego_current_state_.velocity; //bad data from ndt
  // ndt_ego_current_state_.yawRate = gps_ego_current_state_.yawRate;   //bad data from ndt

  static ego_state_ts ndt_ego_prev_state_ = ndt_ego_current_state_;

  ndt_ego_current_state_.velocity = sqrt(pow(ndt_ego_current_state_.mean_x - ndt_ego_prev_state_.mean_x, 2.0) \
										+ pow(ndt_ego_current_state_.mean_y - ndt_ego_prev_state_.mean_y, 2.0))/ndt_ego_current_state_.delta_t;
	ndt_ego_current_state_.yawRate = (ndt_ego_current_state_.mean_yaw - ndt_ego_prev_state_.mean_yaw)/ndt_ego_current_state_.delta_t;

  if(ndt_ego_current_state_.velocity <= 0.3 && ndt_ego_current_state_.velocity >= -0.3)
  {
    ndt_ego_current_state_.yawRate = 0.0;
  }

  // printf("------------------------------------------------------\n");
  // printf("ndt std dev noise px: %f\n", ndt_ego_current_state_.sigmaX);
  // printf("ndt std dev noise py: %f\n", ndt_ego_current_state_.sigmaY);
  // printf("ndt std dev noise yaw: %f\n", ndt_ego_current_state_.sigmaYaw);
  // printf("ndt std dev noise yawd: %f\n", ndt_ego_current_state_.sigmaYawrate);
  // printf("ndt std dev noise vx: %f\n", ndt_ego_current_state_.sigmaVelX);
  // printf("ndt std dev noise vy: %f\n", ndt_ego_current_state_.sigmaVelY);
  // printf("------------------------------------------------------\n");

  printf("------------------------------------------------------\n");
  printf("ndt x: %f\n", ndt_ego_current_state_.mean_x);
  printf("gps x: %f\n", gps_ego_current_state_.mean_x);
  printf("ndt y: %f\n", ndt_ego_current_state_.mean_y);
  printf("gps y: %f\n", gps_ego_current_state_.mean_y);
  printf("ndt yaw: %f\n", ndt_ego_current_state_.mean_yaw);
  printf("gps yaw: %f\n", gps_ego_current_state_.mean_yaw);
  printf("ndt yawrate: %f\n", ndt_ego_current_state_.yawRate);
  printf("gps yawrate: %f\n", gps_ego_current_state_.yawRate);
  printf("gps speed V: %f\n", gps_ego_current_state_.velocity);
  printf("ndt speed V: %f\n", ndt_ego_current_state_.velocity);
  printf("Delta t: %f\n", ndt_ego_current_state_.delta_t);
   
   previous_time = current_time;

  if (!ndt_update_lock_) 
      return;

  if(!init_done_) //Initialize Filter
	{
    if(ndt_ego_current_state_.velocity > 0.3 || ndt_ego_current_state_.velocity < - 0.3)
    {
      //[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      vec_state_ << ndt_ego_current_state_.mean_x, ndt_ego_current_state_.mean_y, 
      ndt_ego_current_state_.velocity, ndt_ego_current_state_.mean_yaw, ndt_ego_current_state_.yawRate; 

      standard_devi_measure_noise_odom_ndt_px_ = 0.1; //gps_ego_current_state_.sigmaX * 100.0 * 5.0;//gps_ego_current_state_.sigmaX; 0.000256 ndt 0.02
      standard_devi_measure_noise_odom_ndt_py_ = 0.1; //gps_ego_current_state_.sigmaY * 100.0 * 5.0;// gps_ego_current_state_.sigmaY; 0.000323 ndt 0.02
      standard_devi_measure_noise_odom_ndt_yaw_ = 0.0006;  //ndt_ego_current_state_.sigmaYaw; 0.00012// gps_ego_current_state_.sigmaYaw; 0.000025
      standard_devi_measure_noise_odom_ndt_yawd_ = 0.1; //gps_ego_current_state_.sigmaYawrate; 0
      standard_devi_measure_noise_odom_ndt_vx_ = 0.1;; //gps_ego_current_state_.sigmaVelX; 0.01
      standard_devi_measure_noise_odom_ndt_vy_ = 0.1; //gps_ego_current_state_.sigmaVelY; 0.01
      ///* Lidar measurement noise covariance matrix
      mat_measure_noise_cov_odom_ndt_(0, 0) = standard_devi_measure_noise_odom_ndt_px_ * standard_devi_measure_noise_odom_ndt_px_;
      mat_measure_noise_cov_odom_ndt_(0, 1) = 0;
      mat_measure_noise_cov_odom_ndt_(1, 0) = 0;
      mat_measure_noise_cov_odom_ndt_(1, 1) = standard_devi_measure_noise_odom_ndt_py_ * standard_devi_measure_noise_odom_ndt_py_;

		  ukf_-> stateLateInitializer(     
      ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        vec_state_,
      ///* Process noise standard deviation longitudinal acceleration in m/s^2
        standard_devi_proc_noise_acceleration_,

      ///* Process noise standard deviation yaw acceleration in rad/s^2
        standard_devi_proc_noise_yaw_,

      ///* Laser measurement noise standard deviation position1 in m
        standard_devi_measure_noise_odom_ndt_px_,

      ///* Laser measurement noise standard deviation position2 in m
        standard_devi_measure_noise_odom_ndt_py_,

        standard_devi_measure_noise_odom_ndt_yaw_,

        standard_devi_measure_noise_odom_ndt_yawd_,

        standard_devi_measure_noise_odom_ndt_vx_,

        standard_devi_measure_noise_odom_ndt_vy_,
       ///* Lidar measurement noise covariance matrix
        mat_measure_noise_cov_odom_ndt_);

		  init_done_= true;
     }
	}
  else if(init_done_)
   {
	  if(ndt_ego_current_state_.velocity > 0.3 || ndt_ego_current_state_.velocity < - 0.3)
    {
      ukf_ -> prediction(ndt_ego_current_state_.delta_t);

      if(use_ndt_ == true) 
      {
          ukf_ -> updateLidarNDT(ndt_ego_current_state_);
          double x_out_ndt = ukf_-> getVecState(0);
          double y_out_ndt = ukf_-> getVecState(1);
          double yaw_out_ndt = ukf_-> getVecState(3);
          printf("UKF filtered yaw updated by NDT: %f\n", yaw_out_ndt);
          visualize_odom(yaw_out_ndt);
      }
    }
	}

  ndt_ego_prev_state_ = ndt_ego_current_state_;
 }

// void EgoMotionTrackingUKF::timeSyncCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ndt_state,
//                                             const nav_msgs::Odometry::ConstPtr& odom_state)
// {

// }

void EgoMotionTrackingUKF::visualize_odom(double yaw)
{
	nav_msgs::Odometry output;
	output.header.frame_id = frame_;
	output.header.stamp = ros::Time::now();
	output.pose.pose.position.x = ukf_-> getVecState(0);
	output.pose.pose.position.y = ukf_-> getVecState(1);
	tf::Matrix3x3 direction_mat; //TODO: Need something better here, without tf
	tf::Quaternion direction_q_tf;
	direction_mat.setRPY(0, 0, yaw); 
	direction_mat.getRotation(direction_q_tf);
	output.pose.pose.orientation.x = direction_q_tf[0];
	output.pose.pose.orientation.y = direction_q_tf[1];
	output.pose.pose.orientation.z = direction_q_tf[2];
	output.pose.pose.orientation.w = direction_q_tf[3];
	fused_state_pub_.publish(output);
}

//node main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  EgoMotionTrackingUKF ego_tracker(nh, priv_nh);
  // go !
  ros::spin();

  return 0;
}// EgoMotionTrackingUKF.cpp

// ------------------------------------------------------
// gps std dev noise px: 0.000149
// gps std dev noise py: 0.000189
// gps std dev noise yaw: 0.000025
// gps std dev noise yawd: 0.000000
// gps std dev noise vx: 0.010000
// gps std dev noise vy: 0.010000
// ------------------------------------------------------
// ndt std dev noise px: 0.020000
// ndt std dev noise py: 0.020000
// ndt std dev noise yaw: 0.000120
// ndt std dev noise yawd: 0.000000
// ndt std dev noise vx: 0.010000
// ndt std dev noise vy: 0.010000
// ------------------------------------------------------