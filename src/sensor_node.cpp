#include "ros/ros.h"
#include "Camera_manager.h"


int main(int argc, char **argv)
{
	ROS_INFO("Sensor node : ON");
	
	ros::init(argc, argv, "sensor"); 
	
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	
	Robotics::GameTheory::Camera_manager l_Camera_manager;
	//Robotics::GameTheory::Robot_manager Manager;
	ros::NodeHandle n;
	while(ros::ok())
	{
	  ros::spinOnce();
	  l_Camera_manager.pack_passage();
	}
	
     
	return 0;
}
