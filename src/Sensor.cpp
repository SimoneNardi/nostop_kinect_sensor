#include "ros/ros.h"

#include "SensorCollector.h"
#include "SensorCollection.h"

int main(int argc, char **argv)
{
	std::cout << "Sensor: before init!" << std::endl << std::flush;
	
	ros::init(argc, argv, "sensor"); 
	
	std::cout << "Sensor: initialize exe"<< std::endl << std::flush;
	
	// Compute Robot Position:
	// From Kinect?ROS_INFO
	// From Simulator?
	Robotics::GameTheory::SensorCollector l_sensorCollector;
	l_sensorCollector.start();
		
	ros::spin();

	return 0;
}
