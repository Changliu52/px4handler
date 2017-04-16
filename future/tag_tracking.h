// Tag tracking library to be used for other projects
// It uses ar_tag and uav local_pose to compute velocity command for the uav


#ifndef __TAG_TRACKING_H__
#define __TAG_TRACKING_H__

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>


ros::Subscriber alvar_sub;

// initiate control variables
double				tag_delta_track_[3]; // feedback control for (x velocity, y velocity, yaw rate)
int				tag_target_id_;
int				tag_target_action_;
const double			velo_ctl_scale_ = 1.0;
	
void q_mult(double* q1, double* q2, double* q3) 
{
  q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
  q3[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
  q3[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];
  q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
}

void QuatRot(double *vec, double *q, double *newVec) // expecting vec is a 4d vector {0, x, y, z}
{
  double q_conj[4];
  q_conj[0] =  q[0];
  q_conj[1] = -q[1];
  q_conj[2] = -q[2];
  q_conj[3] = -q[3];
             
  double temp[4];
  q_mult(q, vec, temp);
  q_mult(temp, q_conj, newVec);
}


// initiate the variables
void tag_init()
{
	tag_delta_track_[0] = 0.0; // feedback control for x-y velocity
	tag_delta_track_[1] = 0.0;
	tag_delta_track_[2] = 0.0;
	tag_target_id_      = 4; // x is the target id
        tag_target_action_  = 0; // y is the target action: 0 is hover tracking, 1 is land, 2 is takeoff
}

void tag_compute(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msgin, geometry_msgs::PoseStamped  px4_pose_)
{
	// See how many tags are detected
	int n_tags = msgin->markers.size();
	
	// Only do this when at least one tag is detected
	if (n_tags > 0 && !isnan(px4_pose_.pose.orientation.w)) {
		// Clear buffer
		tag_delta_track_[0] = 0.0; tag_delta_track_[1] = 0.0; tag_delta_track_[2] = 0.0;

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
				tag_delta_track_[0] = velo_ctl_scale_ * p_tag_in_world_relative_to_px4[1]; //x relative position to the centre of tags
				tag_delta_track_[1] = velo_ctl_scale_ * p_tag_in_world_relative_to_px4[2]; //y relative position to the centre of tags
			}
		}
	}	
	/*
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
	*/
}

#endif