#include "ros/ros.h"

#include "Camera_manager.h"


int main(int argc, char **argv)
{
	ROS_INFO("Acquisition node : ON");
	
	ros::init(argc, argv, "acquisition_node"); 
	
	Robotics::GameTheory::Camera_manager l_Camera_manager;
	ros::NodeHandle n;
	ros::Time begin = ros::Time::now();
	ros::Time now;
	while(ros::ok())
	{
	  ros::spinOnce();
	  now = ros::Time::now();
	  if (now.sec - begin.sec > 1)
	  {
	    l_Camera_manager.reset();
	    begin.sec = now.sec;
	  }
	  l_Camera_manager.pack_passage();
	}
	
     
	return 0;
}
