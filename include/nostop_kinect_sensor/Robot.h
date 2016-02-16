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
#include "Ball_tracker.h"
#include "Robot_manager.h"
#include "Camera.h"
#include "ball_position.h"

namespace Robotics 
{
	namespace GameTheory
	{	
	class Ball_tracker;
	  
	  class Robot
		{
		  ros::NodeHandle m_robot;
		  ros::Publisher m_robot_pose_pub;
		  std::string m_name;
		  std::string m_front_marker_color;
		  std::string m_back_marker_color;
		  
		  // MARKER POSITION
		  ball_position m_front_pos;
		  ball_position m_back_pos;
		  float m_heading;
		  bool found;
		  int m_notFoundCount;
		  cv::Rect m_f_rect;
		  cv::Rect m_b_rect;
		  
		  std::shared_ptr<Ball_tracker> Front_ptr;
		  std::shared_ptr<Ball_tracker> Back_ptr;
		  
		public:

		      void publish_pose(ball_position front_pos,ball_position back_pos,float yaw);
		      void select_robot_pose(std::vector<ball_position>& front_array,std::vector<ball_position>& back_array);
		      std::string color_f();
		      std::string color_b();
		      Robot(std::string name_);
			~Robot();
		};

	}
}


#endif // ROBOT_H
