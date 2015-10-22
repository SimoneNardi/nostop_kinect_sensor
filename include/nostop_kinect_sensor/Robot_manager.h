////////////////////////////////////////////////////////////
//	Robot_manager.h
//	Created on:	22-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H
#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "Robot.h"
namespace Robotics 
{
	namespace GameTheory
	{	
	  class Robot;
		class Robot_manager
		{
		  ros::NodeHandle m_manager_node;
		  ros::Subscriber m_robot_in;
// 		  image_transport::ImageTransport m_man_it;
		  
		  int m_robot_count;
		  std::vector<Robot> m_robot_array;
		  
			void new_robot();
			void update();
			void sub();

		public:
			Robot_manager();
			~Robot_manager();
		};

	}
}


#endif // ROBOT_MANAGER_H