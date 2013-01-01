#ifndef __PX4HANDLER_H__
#define __PX4HANDLER_H__

#include "ros/ros.h"
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <string>

#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
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

#define TAG_ACT_HOVER		0
#define TAG_ACT_LAND		1
#define TAG_ACT_TAKEOFF		2

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
	void reset_setpoint_pose();
	void publish_acceleration_command_from_rc();
	void publish_vision_pose_to_px4();		// converse ekf_state_ to vision_pose/pose (geometry_msgs/PoseStamped) and send out
	void publish_vision_pose_to_px4_smoothed();
	void publish_setpoint_heading_from_rc();
	void publish_setpoint_velo_from_tag();
	
	
	//_____________________________________
	// PRIVATE variables and function
	private:
	Matrix3d test_;
	// internal buffers
	mavros_msgs::ManualControl	rc_command_;		// rc_command_.x, y, z, r in float32; rc_command_.buttons in uint16_t (Note: buttons is used for rc status, refers to #define
	vi_ekf::teensyPilot		ekf_state_;		// ekf_output_.qw, qx, qy, qz, px, py, pz, vx, vy, vz, bx, by, bz, lambda, status, in float, status refers to #define
	geometry_msgs::PoseStamped	vision_pose_;		// ekf_state_ estimation copied in the form of posestamped format
	geometry_msgs::Pose		vision_pose_offset;	// the position and orientation offset to align vision with px4
	geometry_msgs::Pose		vision_pose_aligned;	// the aligned position and orientation measurement from vision to be sent to px4
	geometry_msgs::PoseStamped	px4_pose_;		// ekf_state_ estimation copied in the form of posestamped format
	sensor_msgs::Imu		Imu_;			// imu messurement from pixhawk
	geometry_msgs::TransformStamped	imu_teensy_;		// imu infor from teensy
	geometry_msgs::PoseStamped	setpoint_pose_;		// position command to control the pixhawk
	bool				lock_exposure_;		// We have this for make sure only send system message for once for transation
	// initiate control variables
	double				tag_delta_track_[2]; // feedback control for x-y velocity
	int				tag_target_id_;
	int				tag_target_action_;

	// ROS subscribers
	ros::Subscriber	rclistener_;
	ros::Subscriber	imulistener_;
	ros::Subscriber viekflistener_;
	ros::Subscriber teensylistener_;
	ros::Subscriber px4pose_listener_;
	ros::Subscriber alvarlistener_;
	ros::Subscriber target_tag_listener_;

	// ROS publishers
	ros::Publisher  accelerationcommander_;
	ros::Publisher  accelerationcommander_param_;
	ros::Publisher	vision_pose_pub_;
	ros::Publisher	local_setpoint_pub_;
	ros::Publisher	cmd_vel_pub_;
	ros::Publisher	point_debugger_;
	ros::Publisher 	tag_tracking_debugger_;

	// ROS client
	ros::ServiceClient arming_client_;

	// subscriber callbacks
	void rc_cb		(const mavros_msgs::RCIn::ConstPtr& msgin);
	void imu_cb		(const sensor_msgs::Imu::ConstPtr& msgin);
	void teensy_cb		(const geometry_msgs::TransformStamped::ConstPtr& msgin);
	void viekf_cb		(const vi_ekf::teensyPilot::ConstPtr& msgin);
	void px4pose_cb		(const geometry_msgs::PoseStamped::ConstPtr& msgin);
	void AlvarMarkers_cb	(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msgin);
	void target_tag_cb	(const geometry_msgs::Point::ConstPtr& msgin);
};


#endif
