#ifndef __PX4HANDLER_H__
#define __PX4HANDLER_H__

#include "ros/ros.h"
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <string>

#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <vi_ekf/teensyPilot.h>
#include <std_msgs/Bool.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <px4handler/HelperFunc.h>

#define EKF_NO_ACTION		0
#define EKF_SAFE_OUTPUT		1
#define EKF_REQUEST_AUTO_INIT	2
#define EKF_FAILSAFE		3

#define RC_OFFBOARD		2
#define RC_POSITION		1
#define RC_MANUAL		0

using namespace Eigen;

class px4handler
{
	//_____________________________________
	// PUBLIC variables and function
	public:
	// Constructor
	px4handler(ros::NodeHandle* nh, double loophz);
	double loophz_;
	
	//______________________________________
	// Public Functions
	void interface_vi_ekf();
	void publish_acceleration_command_from_rc();
	void publish_vision_pose_to_px4();		// converse ekf_state_ to vision_pose/pose (geometry_msgs/PoseStamped) and send out
	void publish_vision_pose_to_px4_smoothed();
	void publish_setpoint_heading_from_rc();
	void reset_setpoint_pose();
	
	
	//_____________________________________
	// PRIVATE variables and function
	private:
	Matrix3d test_;
	// internal buffers
	mavros_msgs::ManualControl	rc_command_;		// rc_command_.x, y, z, r in float32; rc_command_.buttons in uint16_t (Note: buttons is used for rc status, refers to #define
	vi_ekf::teensyPilot		ekf_state_;		// ekf_output_.qw, qx, qy, qz, px, py, pz, vx, vy, vz, bx, by, bz, lambda, status, in float, status refers to #define
	geometry_msgs::PoseStamped	vision_pose_;		// ekf_state_ estimation copied in the form of posestamped format
	sensor_msgs::Imu		Imu_;			// imu messurement from pixhawk
	geometry_msgs::PoseStamped	setpoint_pose_;		// position command to control the pixhawk
	bool				lock_exposure_;		// We have this for make sure only send system message for once for transation
	
	// ROS subscribers
	ros::Subscriber	rclistener_;
	ros::Subscriber	imulistener_;
	ros::Subscriber viekflistener_;
	ros::Subscriber alvarlistener_;

	// ROS publishers
	ros::Publisher  accelerationcommander_;
	ros::Publisher  accelerationcommander_param_;
	ros::Publisher	vision_pose_pub_;
	ros::Publisher	local_setpoint_pub_;
	ros::Publisher	cmd_vel_pub_;
	
	// subscriber callbacks
	void rc_cb		(const mavros_msgs::RCIn::ConstPtr& msgin);
	void imu_cb		(const sensor_msgs::Imu::ConstPtr& msgin);
	void viekf_cb		(const vi_ekf::teensyPilot::ConstPtr& msgin);
	void AlvarMarkers_cb	(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msgin);
	
};


#endif