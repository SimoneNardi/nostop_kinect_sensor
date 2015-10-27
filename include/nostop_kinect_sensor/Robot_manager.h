////////////////////////////////////////////////////////////
//	Robot_manager.h
//	Created on:	22-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H
#pragma once
#include "some_struct.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "Robot.h"

#include "nostop_kinect_sensor/Id_robot.h"


namespace Robotics 
{
	namespace GameTheory
	{	
	  class Robot;
	  
	  
		class Robot_manager
		{
		  
		  ros::Subscriber m_robot_in;

		  int m_robot_count;
		  ID * m_robot_id_array = new ID[10];
		  std::shared_ptr<Robot> m_robot_single_ptr =  std::make_shared<Robot>();
		  ID robot_id;
		  
		private:
			ros::NodeHandle m_manager_node;

		public:
			Robot_manager();
			~Robot_manager();
			void subscribe();
			void new_robot_id(const nostop_kinect_sensor::Id_robot::ConstPtr& msg);
			void update();
			void array_assignment(ball_position blue_array[],ball_position green_array[],ball_position red_array[],ball_position yellow_array[],int blue_count,int green_count,int red_count,int yellow_count,cv::Mat stream);
		};

	}
}


#endif // ROBOT_MANAGER_H