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

		  int m_robot_count;
// 		  std::vector<ID> m_robot_id_array;
		  ID * m_robot_id_array = new ID[10];
// 		  std::shared_ptr<Robot>  m_robot_ptr_array(new Robot[10]); 
		  std::shared_ptr<Robot> m_robot_single_ptr =  std::make_shared<Robot>();
		  Robot *m_robot_ptr_array[10];
// 		  std::vector<std::shared_ptr<Robot>> m_robot_ptr_array;
		  ID robot_id;
		  
		private:
			ros::NodeHandle m_manager_node;

		public:
			Robot_manager();
			~Robot_manager();
			void subscribe();
			void new_robot_id(const nostop_kinect_sensor::Id_robot::ConstPtr& msg);
			void update();
			void threshold_update(cv::Mat blue, cv::Mat green, cv::Mat red, cv::Mat yellow, cv::Mat blue_circles_out, cv::Mat green_circles_out, cv::Mat red_circles_out, cv::Mat yellow_circles_out);
		};

	}
}


#endif // ROBOT_MANAGER_H