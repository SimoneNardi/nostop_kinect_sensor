#include "ros/ros.h"
#include "Collector.h"
#include "Collection.h"
#include "Tracker.h"
#include "Robot_manager.h"


int main(int argc, char **argv)
{
	ROS_INFO("Sensor node : ON");
	
	
	
	ros::init(argc, argv, "sensor"); 
	
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	
	Robotics::GameTheory::Collector l_Collector;
	//Robotics::GameTheory::Robot_manager Manager;
	ros::NodeHandle n;
	while(ros::ok())
	{
	  ros::spinOnce();
	  l_Collector.pack_passage();
	}
	
     
	return 0;
}
