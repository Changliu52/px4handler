#include "ros/ros.h"
#include <px4handler/px4handler.h>

//#######################################################
int main(int argc, char **argv)
{
	//_____________________________________
	// ROS node init
	ros::init(argc, argv, "px4handler_node");
	ros::NodeHandle roshandler;
	
	//_____________________________________
	// Main px4 handler init
	double LoopHz = 6.0;
	px4handler px4(&roshandler, LoopHz);
	
	// RUN
	ros::Rate loop_rate(LoopHz); // Hz
	while(ros::ok()) {
		px4.interface_vi_ekf();
		
		// House keeping
		ros::spinOnce();
		loop_rate.sleep();
	} 
	
	return 0;
}