#include "ros/ros.h"
#include "Collector.h"
#include "Collection.h"
#include "Tracker.h"

int main(int argc, char **argv)
{
	ROS_INFO("Sensor node : ON");
	
	
	ros::init(argc, argv, "sensor"); 
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	
	Robotics::GameTheory::Collector l_Collector;
	
	l_Collector.start();
	ros::NodeHandle n;

	ros::spin();
	
	l_Collector.stop();
     
	return 0;
}
