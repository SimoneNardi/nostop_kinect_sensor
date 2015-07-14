#include "ros/ros.h"

#include "SensorCollector.h"
#include "SensorCollection.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor"); 
	
	// Compute Robot Position:
	// From Kinect?
	// From Simulator?
	Robotics::GameTheory::SensorCollector l_sensorCollector;
	l_sensorCollector.start();
		
	ros::spin();

	return 0;
}
