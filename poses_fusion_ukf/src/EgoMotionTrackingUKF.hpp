// EgoMotionTrackingUKF.h
// Unscented Kalman Filter for sensor fusion node
// Created on: 2020.02.17
// Author: Chao Li
// Email: chao.li.arthur@gmail.com

#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "containers/data_container.h"
#include "filters/UnscentedKalmanFilter.h"

class EgoMotionTrackingUKF
{       
  public:
    EgoMotionTrackingUKF(
	  ros::NodeHandle nh,
	  ros::NodeHandle priv_nh
	);

	~EgoMotionTrackingUKF();

    // odometry callback function.
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_state); 
	// ndt callback function.
	void ndtCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ndt_state);
    // message filter callback fucntion
    //void timeSyncCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ndt_state, const nav_msgs::Odometry::ConstPtr& odom_state);

	// predic timer call back funchtion
	void timerCallback(const ros::TimerEvent&);

	//visualize odometry function.
	void visualize_odom(double yaw);
	void visualize_particles();

  private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;
	bool init_done_;
	bool use_odom_;
    bool use_ndt_;
    bool first_predict_done_;
    bool gps_update_lock_;
    bool ndt_update_lock_;


    ///* State dimension
    int state_dimension_;  //n_x

    ///* Sigma points dimension
    int sigma_points_dimension_;

    ///* Augmented state dimension
    int augmented_dimension_;  //n_aug

    ///* state vector: [pos_x pos_y vel_abs yaw_angle yaw_rate] in SI units and rad
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
    bool lock_;

	ego_state_ts gps_ego_current_state_;
    ego_state_ts ndt_ego_current_state_;

	double velocity_global_;
	double theta_global_;
	double prediction_freq_;
    ros::Timer timer_;

	std::string frame_; 
	double covariance_yawrate_; 
	UKF *ukf_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *ndt_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped,
            nav_msgs::Odometry> SyncTrackPolicyT;
    message_filters::Synchronizer<SyncTrackPolicyT> *track_synchronizer_;
    ros::Publisher fused_state_pub_;
};





