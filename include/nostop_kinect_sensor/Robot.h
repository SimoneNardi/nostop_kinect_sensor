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
#include "Tracker.h"

#include "nostop_kinect_sensor/Id_robot.h"


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
		  float m_front_pos[2];
		  float m_back_pos[2];
		  Tracker front_marker;
		  Tracker back_marker;
		  std::shared_ptr<Tracker> Front_ptr;
		  std::shared_ptr<Tracker> Back_ptr;
		  
		  cv::Mat m_Front,m_Back;
		public:
		      void pose_heading();
		      void marker_selection();
		      void pubID();
		      void takeImgFront(cv::Mat img);
		      void takeImgBack(cv::Mat img);
			Robot();
			~Robot();
		};

	}
}


#endif // ROBOT_H