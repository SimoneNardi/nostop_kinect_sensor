////////////////////////////////////////////////////////////
//	Robot.h
//	Created on:	22-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef ROBOT_H
#define ROBOT_H
#pragma once
#include <string>
#include <ros/ros.h>
#include "some_struct.h"
#include "Tracker.h"
#include "Robot_manager.h"
#include "Collection.h"
#include "nostop_agent/Id_robot.h"


namespace Robotics 
{
	namespace GameTheory
	{	
	class Tracker;
	  
	  class Robot
		{
		  ros::NodeHandle m_robot;
		  ros::Publisher m_robot_pub;

		  std::string m_name;
		  std::string m_front_marker_color;
		  std::string m_back_marker_color;
		 
		  // MARKER POSITION
		  ball_position m_front_pos;
		  ball_position m_back_pos;
		  float m_heading;
		  bool found;
		  bool m_notFoundCount;
		  cv::Rect m_f_rect;
		  cv::Rect m_b_rect;
		  
		  std::shared_ptr<Tracker> Front_ptr;
		  std::shared_ptr<Tracker> Back_ptr;
		  
		public:

		      void pubID();
		      void select_robot_pose(std::vector<ball_position>& front_array,std::vector<ball_position>& back_array);
		      void draw_circles(cv::Mat src);// NOT IN USE
		      std::string color_f();
		      std::string color_b();
		      Robot(std::string name_);
			~Robot();
		};

	}
}


#endif // ROBOT_H