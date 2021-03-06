#include <px4handler/px4handler.h>

// Constructor
px4handler::px4handler(ros::NodeHandle* nh, double loophz)
{
	test_ = Matrix3d::Zero();
	loophz_ = loophz;
	
	// for camera exposure
	lock_exposure_ = 0;
	system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure false");
	
	// for tag
	tag_delta_track_[0] = 0.0; // feedback control for x-y velocity
	tag_delta_track_[1] = 0.0;
	tag_target_id_      = 4; // x is the target id
        tag_target_action_  = 0; // y is the target action: 0 is hover tracking, 1 is land, 2 is takeoff

	//init subscriber
	rclistener_			= nh->subscribe <mavros_msgs::RCIn>			("mavros/rc/in",		1, &px4handler::rc_cb,			this);
	px4pose_listener_		= nh->subscribe <geometry_msgs::PoseStamped>		("mavros/local_position/pose",	1, &px4handler::px4pose_cb,		this);
	viekflistener_			= nh->subscribe <vi_ekf::teensyPilot>			("ekf/output",			1, &px4handler::viekf_cb,		this);
	alvarlistener_			= nh->subscribe <ar_track_alvar_msgs::AlvarMarkers>	("ar_pose_marker",		1, &px4handler::AlvarMarkers_cb,	this);
	teensylistener_			= nh->subscribe <geometry_msgs::TransformStamped>	("teensy/imu",			1, &px4handler::teensy_cb,		this);
	target_tag_listener_		= nh->subscribe <geometry_msgs::Point>			("px4handler/tag_target",	1, &px4handler::target_tag_cb,		this);

	//init publisher
	accelerationcommander_		= nh->advertise <geometry_msgs::Vector3Stamped>	("mavros/setpoint_accel/accel",		1);
	accelerationcommander_param_	= nh->advertise <std_msgs::Bool>		("mavros/setpoint_accel/send_force",	1);
	vision_pose_pub_		= nh->advertise <geometry_msgs::PoseStamped>	("mavros/vision_pose/pose",		1);
	local_setpoint_pub_		= nh->advertise <geometry_msgs::PoseStamped>	("mavros/setpoint_position/local",	1);
	cmd_vel_pub_			= nh->advertise <geometry_msgs::TwistStamped>	("mavros/setpoint_velocity/cmd_vel",	1);
	point_debugger_			= nh->advertise <geometry_msgs::PointStamped>	("px4handler/debug/point",		1);
	tag_tracking_debugger_		= nh->advertise <geometry_msgs::Point>		("px4handler/debug/tag_target",		1);

	// init clients
	arming_client_ 			= nh->serviceClient <mavros_msgs::CommandBool> ("mavros/cmd/arming");
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
		//publish_setpoint_heading_from_rc();
		if (lock_exposure_ == 0){
			system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure true");
			lock_exposure_ = 1;
		}
	// there theRC is in manual mode
	}else{
		//publish_setpoint_heading_from_rc(); // publish position-heading setpoint even not in offboard mode (this is to avoid offboard mode being rejected
		reset_setpoint_pose();
		if (lock_exposure_ == 1){
			system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure false");
			lock_exposure_ = 0;
		}
	}
	
	// publish velocity control feedback from tag
	publish_setpoint_velo_from_tag();
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

void px4handler::publish_setpoint_velo_from_tag()
{
	// construct basic loiter command
	double velo_ctl_scale = 1.0;
	geometry_msgs::TwistStamped target_velocity;
	target_velocity.header.stamp	= ros::Time::now();
	target_velocity.twist.linear.x	= velo_ctl_scale * tag_delta_track_[0];
	target_velocity.twist.linear.y	= velo_ctl_scale * tag_delta_track_[1];
	target_velocity.twist.linear.z	= 0.0;
	target_velocity.twist.angular.x	= 0.0;//q_world_to_cam[0];
	target_velocity.twist.angular.y = 0.0;//p_tag_in_world_relative_to_px4[1];
	target_velocity.twist.angular.z = 0.0;//Imu_.orientation.w;

	// do reasoning
	if(tag_target_action_ == TAG_ACT_TAKEOFF) {
		
	} else if(tag_target_action_ == TAG_ACT_LAND) {
		target_velocity.twist.linear.z  = -0.25;
	}

	// send command
	cmd_vel_pub_.publish(target_velocity);
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

// Callback function for teensy imu
void px4handler::teensy_cb(const geometry_msgs::TransformStamped::ConstPtr& msgin)
{
	imu_teensy_ = *msgin;
}

// Callback function for ar_tracker_alvar message
void px4handler::AlvarMarkers_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msgin)
{	
	// See how many tags are detected
	int n_tags = msgin->markers.size();
	
	// Only do this when at least one tag is detected
	if (n_tags > 0 && !isnan(px4_pose_.pose.orientation.w)) {
		// Clear buffer
		tag_delta_track_[0] = 0.0; tag_delta_track_[1] = 0.0;

		// Get the camera rotation in world frame
		double q_px4_to_cam[4] = {0.0, 1.0, 0.0, 0.0};
		double q_world_to_px4[4] = {	px4_pose_.pose.orientation.w, //Imu_.orientation.w, //px4_pose_.pose.orientation.w,  //imu_teensy_.transform.rotation.w, 
						px4_pose_.pose.orientation.x, //Imu_.orientation.x, //px4_pose_.pose.orientation.x,  //imu_teensy_.transform.rotation.x, 
						px4_pose_.pose.orientation.y, //Imu_.orientation.y, //px4_pose_.pose.orientation.y,  //imu_teensy_.transform.rotation.y, 
						px4_pose_.pose.orientation.z}; //Imu_.orientation.z}; //px4_pose_.pose.orientation.z}; //imu_teensy_.transform.rotation.z
		double q_world_to_cam[4];
		q_mult(q_world_to_px4, q_px4_to_cam, q_world_to_cam);
	
		// for each detected marker get their position relative to the px4
		for (int i=0; i<n_tags; i++) {
			int tag_id = msgin->markers[i].id;
			if (tag_id == tag_target_id_){
				double p_tag_in_cam[4] = {0.0,	msgin->markers[i].pose.pose.position.x,
								msgin->markers[i].pose.pose.position.y,
								msgin->markers[i].pose.pose.position.z};
				double p_tag_in_world_relative_to_px4[4];
				QuatRot(p_tag_in_cam,
					q_world_to_cam,
					p_tag_in_world_relative_to_px4);
				
				// average to find the centre
				tag_delta_track_[0] = p_tag_in_world_relative_to_px4[1]; //x relative position to the centre of tags
				tag_delta_track_[1] = p_tag_in_world_relative_to_px4[2]; //y relative position to the centre of tags
			}
		}
	}	
	
	// for debugging
	if (point_debugger_.getNumSubscribers()>0) {
		geometry_msgs::PointStamped	tag_delta;		// the velocity feedback from tag tracking
		tag_delta.header.stamp	= ros::Time::now();
		tag_delta.header.frame_id	= "debug";
		tag_delta.point.x = tag_delta_track_[0];
		tag_delta.point.y = tag_delta_track_[1];
		tag_delta.point.z = 0.0;
		point_debugger_.publish(tag_delta);
	}
	if (tag_tracking_debugger_.getNumSubscribers()>0) {
		geometry_msgs::Point	output;
		output.x = tag_target_id_;
		output.y = tag_target_action_;
		tag_tracking_debugger_.publish(output);
	}
}

// Callback function to update the target tag id
void px4handler::target_tag_cb(const geometry_msgs::Point::ConstPtr& msgin)
{
	tag_target_id_ 		= (int)msgin->x; // x is the target id
	tag_target_action_ 	= (int)msgin->y; // y is the target action: 0 is hover tracking, 1 is land, 2 is takeoff
}
