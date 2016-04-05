////////////////////////////////////////////////////////////
//	Robot_manager.h
//	Created on:	22-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H
#pragma once
#include "Robot.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>

#include <vector>
#include "ball_position.h"

namespace Robotics 
{
	namespace GameTheory
	{	
	  class Robot;
	  
	  
		class Robot_manager
		{
			double m_lat0,m_lon0;
			std::vector< std::shared_ptr<Robot> > m_robot_array;
			ros::NodeHandle m_manager_node;

		public:
			Robot_manager(double& lat,double& lon);
			~Robot_manager();

			void array_assignment(std::vector<ball_position>& blue_array, 
						std::vector<ball_position>& green_array,
						std::vector<ball_position>& red_array,
						std::vector<ball_position>& yellow_array);
			
			void add_robot(const std_msgs::String::ConstPtr& msg);
		};
	}
}


#endif // ROBOT_MANAGER_H