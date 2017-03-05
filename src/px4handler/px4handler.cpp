#include <px4handler/px4handler.h>

// Constructor
px4handler::px4handler(ros::NodeHandle* nh, double loophz)
{
	test_ = Matrix3d::Zero();
	loophz_ = loophz;
	
	// for camera exposure
	lock_exposure_ = 0;
	system("rosrun dynamic_reconfigure dynparam set /ueye_cam_nodelet lock_exposure false");
	
	//init subscriber
	rclistener_			= nh->subscribe <mavros_msgs::RCIn>	("mavros/rc/in", 1, &px4handler::rc_cb,    this);
	viekflistener_			= nh->subscribe <vi_ekf::teensyPilot>	("/ekf/output",  1, &px4handler::viekf_cb, this);
	
	//init publisher
	accelerationcommander_		= nh->advertise <geometry_msgs::Vector3Stamped>	("mavros/setpoint_accel/accel",		1);
	accelerationcommander_param_	= nh->advertise <std_msgs::Bool>		("mavros/setpoint_accel/send_force",	1);
	vision_pose_pub_		= nh->advertise <geometry_msgs::PoseStamped>	("mavros/vision_pose/pose",		1);
	local_setpoint_pub_		= nh->advertise <geometry_msgs::PoseStamped>	("mavros/setpoint_position/local",	1);
}


//__________________________________________________________
// Public functions #######################################
// function to make high level decision and make smooth transation between states
// this function runs in the main loop speed
void px4handler::interface_vi_ekf()
{
	// publish vision pose in the undisrupted way
	if (ekf_state_.status == EKF_SAFE_OUTPUT) {
		publish_vision_pose_to_px4_smoothed();
	}
	
	// switching to offboard mode from RC
	if (rc_command_.buttons == RC_OFFBOARD || rc_command_.buttons == RC_POSITION)	{
		publish_setpoint_heading_from_rc();
		if (lock_exposure_ == 0){
			system("rosrun dynamic_reconfigure dynparam set /ueye_cam_nodelet lock_exposure true");
			lock_exposure_ = 1;
		}
	// there theRC is in manual mode
	}else{
		publish_setpoint_heading_from_rc(); // publish position-heading setpoint even not in offboard mode (this is to avoid offboard mode being rejected
		reset_setpoint_pose();
		if (lock_exposure_ == 1){
			system("rosrun dynamic_reconfigure dynparam set /ueye_cam_nodelet lock_exposure false");
			lock_exposure_ = 0;
		}
	}
	
	/*
	// when vision is available perform inteligent control
	if (ekf_state_.status == EKF_SAFE_OUTPUT && rc_command_.buttons == RC_OFFBOARD) {
		publish_vision_pose_to_px4();
		publish_setpoint_heading_from_rc();
		
	// when vision tracking false perform direct manual control 	
	} else {
		// direct in rc manual control
		publish_acceleration_command_from_rc();
		// reset position command to the current position
		reset_setpoint_pose();
		//std::cout << acceleration_command(0) << std::endl;
	}
	*/
}


// function to publish xy acceleration and thrust from rc
void px4handler::publish_acceleration_command_from_rc()
{
	std_msgs::Bool sendforce; sendforce.data = false;
	accelerationcommander_param_.publish(sendforce);
	
	geometry_msgs::Vector3Stamped accel;
	accel.header.stamp	= ros::Time::now();
	accel.header.frame_id	= "1"; // in global frame ?????????????????????????????Need to check with a real flight
	accel.vector.x		= (double)rc_command_.x*0.1; // looks like accel is in g unit
	accel.vector.y		= (double)rc_command_.y*0.1;
	accel.vector.z		= (double)rc_command_.z; // it turns out z is throttle command
	accelerationcommander_.publish(accel);
	//std::cout << accel.vector.x << std::endl;
}


// function to publish vision pose for drone
void px4handler::publish_vision_pose_to_px4()
{	
	vision_pose_pub_.publish(vision_pose_);
}

// function to publish vision pose for drone in the artificially smoothed way
void px4handler::publish_vision_pose_to_px4_smoothed()
{
	geometry_msgs::PoseStamped		vision_pose_smoothed;
	vision_pose_smoothed			= vision_pose_;	
	// timestamp needs to be updated to fool the px4	
	vision_pose_smoothed.header.stamp	= ros::Time::now();
	// publish vision pose
	vision_pose_pub_.publish(vision_pose_smoothed);
}


// function to publishe position setpoint and heading command from rc
void px4handler::publish_setpoint_heading_from_rc()
{	
	// simulate xy velocity from rc
	setpoint_pose_.pose.position.x += (double)rc_command_.x / loophz_;
	setpoint_pose_.pose.position.y += (double)rc_command_.y / loophz_;
	setpoint_pose_.pose.position.z += (double)rc_command_.z / loophz_;
	// TODO should also do heading control
	
	// Publish local pose setpoint
	local_setpoint_pub_.publish(setpoint_pose_);
}


// function to reset position setpoint to align with current vision pose measurment
void px4handler::reset_setpoint_pose()
{
	setpoint_pose_ = vision_pose_;
}



//____________________________________________________________
// Local callbacks  #########################################
// Callback function when hear "mavros/rc/in"
void px4handler::rc_cb(const mavros_msgs::RCIn::ConstPtr& msgin)
{
	// only do if RC signal strength is high enough
	if (msgin->rssi >= 10) {
		// scale to -1.0 to 1.0
		rc_command_.x		= ((float)msgin->channels[1]-1500.0)/510.0;
		rc_command_.y		= ((float)msgin->channels[0]-1500.0)/510.0;
		rc_command_.z		= ((float)msgin->channels[2]-1500.0)/510.0;
		rc_command_.r		= ((float)msgin->channels[3]-1500.0)/510.0;
		
		// remove small imprefection drifting
		if (rc_command_.x > -0.02 && rc_command_.x < 0.02) rc_command_.x = 0.0;
		if (rc_command_.y > -0.02 && rc_command_.y < 0.02) rc_command_.y = 0.0;
		if (rc_command_.r > -0.02 && rc_command_.r < 0.02) rc_command_.r = 0.0;
		
		// offboard mode detection
		if	((float)msgin->channels[4] > 1800.0)	rc_command_.buttons = RC_OFFBOARD;
		else if ((float)msgin->channels[4] > 1600.0)	rc_command_.buttons = RC_POSITION;
		else						rc_command_.buttons = RC_MANUAL;
		
	// RC lost if signal strength is too low
	} else {
		rc_command_.buttons	= RC_MANUAL;
	}
}

// Callback function when hear 
void px4handler::imu_cb(const sensor_msgs::Imu::ConstPtr& msgin)
{
	Imu_ = *msgin;
}

// Callback function when hear "/ekf/output"
void px4handler::viekf_cb(const vi_ekf::teensyPilot::ConstPtr& msgin)
{
	ekf_state_ = *msgin;
	if (ekf_state_.status == EKF_SAFE_OUTPUT) {
		vision_pose_.header.stamp	= ros::Time::now();
		vision_pose_.header.frame_id	= "fcu";
		vision_pose_.pose.orientation.w	= (double)ekf_state_.qw;
		vision_pose_.pose.orientation.x	= (double)ekf_state_.qx;
		vision_pose_.pose.orientation.y	= (double)ekf_state_.qy;
		vision_pose_.pose.orientation.z	= (double)ekf_state_.qz;
		vision_pose_.pose.position.x	= (double)ekf_state_.px;
		vision_pose_.pose.position.y	= (double)ekf_state_.py;
		vision_pose_.pose.position.z	= (double)ekf_state_.pz;
	}
}