#include "ros/ros.h"

#include "Camera_manager.h"


int main(int argc, char **argv)
{
	ROS_INFO("Acquisition node : ON");
	
	ros::init(argc, argv, "acquisition_node"); 
	
	Robotics::GameTheory::Camera_manager l_Camera_manager;
	ros::NodeHandle n;
	
	while(ros::ok())
	{
	  ros::spinOnce();
	  l_Camera_manager.pack_passage();
	}
	
     
	return 0;
}
