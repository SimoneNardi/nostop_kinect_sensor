#include "ros/ros.h"
#include "Camera_manager.h"


int main(int argc, char **argv)
{
	ROS_INFO("Acquisition node : ON");
	
	ros::init(argc, argv, "acquisition_node"); 
	
	ros::NodeHandle n("~");
	int frequency;
	double lat0,lon0;
	n.param("frequency",frequency,50);
	n.param("SRworld_lat0",lat0,47.0);
	n.param("SRworld_lon0",lon0,-10.0);
	Robotics::GameTheory::Camera_manager l_Camera_manager(lat0,lon0);

	ros::Rate r(frequency);
	while(ros::ok())
	{
	  l_Camera_manager.pack_passage();
	  ros::spinOnce();
	  r.sleep();
	}
	
     
	return 0;
}
