
#ifndef M7_TRACKER_INCLUDE_GROUND_VEHICLE_H_
#define M7_TRACKER_INCLUDE_GROUND_VEHICLE_H_

#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/Image.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <vector>
#include <string.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>

#define ROOMBA_HEIGHT 0.09
#define TIME_STEP 0.1

class GroundVehicle
{
public:
	GroundVehicle(double dt,
					 const Eigen::Matrix4d& F,
					 const Eigen::Matrix<double, 2, 4>& H,
					 const Eigen::Matrix4d& Q,
					 const Eigen::Matrix<double, 2, 2>& R,
					 const Eigen::Matrix4d& P);

	GroundVehicle();
	void init(double t0, const Eigen::Matrix<double, 4, 1>& x0);
	void predict();
	void update(const Eigen::Matrix<double, 2, 1>& y, std_msgs::Header imageHeader);

private:
	 /**
	  * Create a Kalman filter with the specified matrices.
	  *   F - System dynamics matrix
	  *   H - Output matrix (Gaussian measurement combination matrix)
	  *   Q - Process noise covariance
	  *   R - Measurement noise covariance
	  *   P - Estimated error covariance
	  *   K - Kalman Gain
	  */

	Eigen::Matrix4d F, Q, P;
	Eigen::Matrix<double, 2, 4> H;
	Eigen::Matrix<double, 2, 2> R;
	Eigen::Matrix<double, 4, 2> K;

	//Initial and current time
	double t0, t;

	//Time Step
	const double timeStep;
	//Time since Last y
	double dt;

	//Identity
	Eigen::Matrix4d I;

	//Filter Initialized?
	bool initialized;

	//Estimated States
	Eigen::Matrix<double, 4, 1> x_hat, x_hat_new;

	//Time stamp header
	std_msgs::Header dataHeader;
};
