////////////////////////////////////////////////////////////
//	Robot_manager.h
//	Created on:	22-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H
#pragma once
#include "some_struct.h"
#include "Robot.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>


#include "nostop_agent/Id_robot.h"

#include <vector>


namespace Robotics 
{
	namespace GameTheory
	{	
	  class Robot;
	  
	  
		class Robot_manager
		{
		  
		  ros::Subscriber m_robot_in;

		  std::vector< std::shared_ptr<Robot> > m_robot_array;
		  
		private:
			ros::NodeHandle m_manager_node;

		public:
			Robot_manager();
			~Robot_manager();
			void new_robot_id(const nostop_agent::Id_robot::ConstPtr& msg);
			void array_assignment(std::vector<ball_position>& blue_array, 
						std::vector<ball_position>& green_array,
						std::vector<ball_position>& red_array,
						std::vector<ball_position>& yellow_array);
		};

	}
}


#endif // ROBOT_MANAGER_H