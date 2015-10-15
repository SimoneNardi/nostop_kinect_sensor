#include "ros/ros.h"
#include "Collector.h"
#include "Collection.h"
#include "Tracker.h"

void ShutDown(const ros::TimerEvent&)
{
 ros::shutdown();
}


int main(int argc, char **argv)
{
	ROS_INFO("Sensor: init!");
	
	ros::init(argc, argv, "sensor"); 
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	
	Robotics::GameTheory::Collector l_Collector;
	
	l_Collector.start();
     
	ros::NodeHandle n;

// 	ros::Timer l_timer=n.createTimer(ros::Duration(100000), ShutDown);
	ros::spin();
	
	l_Collector.stop();
     
	return 0;
}
