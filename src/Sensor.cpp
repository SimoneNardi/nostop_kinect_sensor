#include "ros/ros.h"

#include "SensorCollector.h"
#include "SensorCollection.h"

int main(int argc, char **argv)
{
	ROS_INFO("Sensor: init!");
	
	ros::init(argc, argv, "sensor"); 
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	Robotics::GameTheory::SensorCollector l_sensorCollector;
	l_sensorCollector.start();
		
	ros::spin();

	return 0;
}
