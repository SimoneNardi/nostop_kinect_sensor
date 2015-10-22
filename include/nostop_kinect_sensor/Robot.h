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

namespace Robotics 
{
	namespace GameTheory
	{	
	class Tracker;
		class Robot
		{
		  ros::NodeHandle m_robot;
		  ros::Publisher m_robot_pub;
		  
		  std::string Name;
		  
		  Tracker front_marker;
		  Tracker back_marker;
		  

		void pose_heading();
		void marker_selection();
		public:
			Robot();
			~Robot();
		};

	}
}


#endif // ROBOT_H