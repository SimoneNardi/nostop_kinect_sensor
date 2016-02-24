#include "ros/ros.h"
#include "Fake_imu.h"

int main(int argc, char **argv)
{
	ROS_INFO("Fake IMU : ON");
	
	ros::init(argc, argv, "fake_imu");
	ros::NodeHandle n("~");
	ros::NodeHandle node;
	std::string robot_name;
	int freq;
	n.getParam("robot_name",robot_name);
	n.getParam("frequency",freq);
	Robotics::GameTheory::Fake_imu fake_imu(robot_name);
	ros::Rate r(freq);
	while(ros::ok())
	{
	  fake_imu.imu_published();
	  r.sleep();
	  ros::spinOnce();
	}
	return 0;
}
