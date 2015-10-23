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

#include "nostop_kinect_sensor/Id_robot.h"


namespace Robotics 
{
	namespace GameTheory
	{	
	  class Robot;
	  struct ID{
	    std::string name;
	    std::string front_marker_color;
	    std::string back_marker_color;
	  };
	  
		class Robot_manager
		{
		  
		  ros::Subscriber m_robot_in;
// 		  image_transport::ImageTransport m_man_it;
		  int m_robot_count;
		  std::vector<ID> m_robot_id_array;
		  std::vector<std::shared_ptr<Robot>> m_robot_ptr_array;
		  std::shared_ptr<Robot> m_robot_single_ptr;
		  
		  ID robot_id;
		  
		private:
			ros::NodeHandle m_manager_node;

		public:
			Robot_manager();
			~Robot_manager();
			void new_robot_id(const nostop_kinect_sensor::Id_robot::ConstPtr& msg);
			void update();
			void threshold_update(cv::Mat blue, cv::Mat green, cv::Mat red, cv::Mat yellow);
		};

	}
}


#endif // ROBOT_MANAGER_H