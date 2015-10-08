#include "ros/ros.h"

#include "Collector.h"
#include "Collection.h"

int main(int argc, char **argv)
{
	ROS_INFO("Sensor: init!");
	
	ros::init(argc, argv, "sensor"); 
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	Robotics::GameTheory::Collector l_Collector;
	l_Collector.start();
	
	for(int i = 1;i<1e4;i++) // TO DO IN SECOND
	{
	  ros::spinOnce();
	}
	l_Collector.stop();
	return 0;
}
