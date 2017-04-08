#include <px4handler/px4handler.h>

// Constructor
px4handler::px4handler(ros::NodeHandle* nh, double loophz)
{
	test_ = Matrix3d::Zero();
	loophz_ = loophz;
	
	// for camera exposure
	lock_exposure_ = 0;
	system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure false");
	
	//init subscriber
	rclistener_			= nh->subscribe <mavros_msgs::RCIn>			("mavros/rc/in",		1, &px4handler::rc_cb,			this);
	px4pose_listener_		= nh->subscribe <geometry_msgs::PoseStamped>		("mavros/local_position/pose",	1, &px4handler::px4pose_cb,		this);
	viekflistener_			= nh->subscribe <vi_ekf::teensyPilot>			("ekf/output",			1, &px4handler::viekf_cb,		this);
	alvarlistener_			= nh->subscribe <ar_track_alvar_msgs::AlvarMarkers>	("ar_pose_marker",		1, &px4handler::AlvarMarkers_cb,	this);
	
	//init publisher
	accelerationcommander_		= nh->advertise <geometry_msgs::Vector3Stamped>	("mavros/setpoint_accel/accel",		1);
	accelerationcommander_param_	= nh->advertise <std_msgs::Bool>		("mavros/setpoint_accel/send_force",	1);
	vision_pose_pub_		= nh->advertise <geometry_msgs::PoseStamped>	("mavros/vision_pose/pose",		1);
	local_setpoint_pub_		= nh->advertise <geometry_msgs::PoseStamped>	("mavros/setpoint_position/local",	1);
	cmd_vel_pub_			= nh->advertise <geometry_msgs::TwistStamped>	("mavros/setpoint_velocity/cmd_vel",	1);
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
			system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure true");
			lock_exposure_ = 1;
		}
	// there theRC is in manual mode
	}else{
		publish_setpoint_heading_from_rc(); // publish position-heading setpoint even not in offboard mode (this is to avoid offboard mode being rejected
		reset_setpoint_pose();
		if (lock_exposure_ == 1){
			system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure false");
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
		// rotate coordinate frame (180 degree rotation about world x axis)
		double q[4] = {(double)ekf_state_.qw, (double)ekf_state_.qx, (double)ekf_state_.qy, (double)ekf_state_.qz};
		double p[4] = {0.0,		      (double)ekf_state_.px, (double)ekf_state_.py, (double)ekf_state_.pz};
		double frame_rot[4] = {0.0, 1.0, 0.0, 0.0}; // 180 degree rotation about world x axis
		
		double p_corrected[4]; // position
		QuatRot(p, frame_rot, p_corrected); // frame_rot*p*frame_rot^(-1) = p_corrected
		
		double q_corrected[4];
		q_mult(q, frame_rot, q_corrected); // do frame rotate first then to q rotate, (we are rotating the frame_rot by q) q*frame_rot=qcorrected
		
		// generate output message
		vision_pose_.header.stamp	= ros::Time::now();
		vision_pose_.header.frame_id	= "fcu";
		vision_pose_.pose.orientation.w	= q_corrected[0];
		vision_pose_.pose.orientation.x	= q_corrected[1];
		vision_pose_.pose.orientation.y	= q_corrected[2];
		vision_pose_.pose.orientation.z	= q_corrected[3];
		vision_pose_.pose.position.x	= p[1];
		vision_pose_.pose.position.y	= p[2];
		vision_pose_.pose.position.z	= p[3];
		
		//vision_pose_.pose.position.x	= p_corrected[1];
		//vision_pose_.pose.position.y	= p_corrected[2];
		//vision_pose_.pose.position.z	= p_corrected[3];
		
		//vision_pose_.pose.orientation.w	= (double)ekf_state_.qw;
		//vision_pose_.pose.orientation.x	= (double)ekf_state_.qx;
		//vision_pose_.pose.orientation.y	= (double)ekf_state_.qy;
		//vision_pose_.pose.orientation.z	= (double)ekf_state_.qz;
		//vision_pose_.pose.position.x	= (double)ekf_state_.px;
		//vision_pose_.pose.position.y	= (double)ekf_state_.py;
		//vision_pose_.pose.position.z	= (double)ekf_state_.pz;
	}
}


// Callback function for local pose topic
void px4handler::px4pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msgin)
{
	px4_pose_ = *msgin;
}


// Callback function for ar_tracker_alvar message
void px4handler::AlvarMarkers_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msgin)
{
	// initiate control variables
	double delta_track[2] = {0.0, 0.0}; // feedback control for x-y velocity
	
	// See how many tags are detected
	int n_tags = sizeof(msgin->markers)/sizeof(msgin->markers[0]);
	
	// Only do this when at least one tag is detected
	if (n_tags > 0) {
		// Get the camera rotation in world frame
		double q_px4_to_cam[4] = {0.0, 1.0, 0.0, 0.0};
		double q_world_to_px4[4] = {	px4_pose_.pose.orientation.w,
						px4_pose_.pose.orientation.x, 
						px4_pose_.pose.orientation.y, 
						px4_pose_.pose.orientation.z};
		double q_world_to_cam[4];
		q_mult(q_world_to_px4, q_px4_to_cam, q_world_to_cam);
	
		// for each detected marker get their position relative to the px4
		for (int i=0; i<n_tags; i++) {
			int tag_id = msgin->markers[i].id;
			double p_tag_in_cam[4] = {0.0,	msgin->markers[i].pose.pose.position.x, 
							msgin->markers[i].pose.pose.position.y, 
							msgin->markers[i].pose.pose.position.z};
			double p_tag_in_world_relative_to_px4[4];
			QuatRot(p_tag_in_cam, 
				q_world_to_cam,
				p_tag_in_world_relative_to_px4);
				
			// average to find the centre
			delta_track[0] += (p_tag_in_world_relative_to_px4[1] / (double)n_tags); //x relative position to the centre of tags
			delta_track[1] += (p_tag_in_world_relative_to_px4[2] / (double)n_tags); //y relative position to the centre of tags
		}
	}
	// publish transformation
	geometry_msgs::TwistStamped target_velocity;
	target_velocity.header.stamp	= ros::Time::now();
	target_velocity.twist.linear.x	= delta_track[0];
	target_velocity.twist.linear.y	= delta_track[1];
	target_velocity.twist.linear.z	= 1.0;
	cmd_vel_pub_.publish(target_velocity);
}