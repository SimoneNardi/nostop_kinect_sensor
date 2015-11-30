////////////////////////////////////////////////////////////
//	Camera_manager.h
//	Created on:	07-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H
#pragma once

// #include "ThreadBase.h"

#include "ros/ros.h"
#include <std_msgs/String.h>

#include <memory>

// #include "Threads.h"
#include "nostop_kinect_sensor/Camera_data.h"
#include "Camera.h"
#include "ball_position.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class Camera;
		
		class Camera_manager
		{

		  Mutex m_mutex;

		  ros::Publisher m_pub;
		  ros::Subscriber m_camera_in;
		  std::vector< std::shared_ptr<Camera> > m_camera_array;
		  std::shared_ptr<Robot_manager> m_manager;
		  
		  // PACKAGE
		  std::vector<ball_position> m_blue_ball_W;
		  std::vector<ball_position> m_green_ball_W;
		  std::vector<ball_position> m_red_ball_W;
		  std::vector<ball_position> m_yellow_ball_W;

		  ros::NodeHandle m_node;
		
		public:
			Camera_manager();
			~Camera_manager();
			void new_camera(const nostop_kinect_sensor::Camera_data::ConstPtr& msg);
 			void pack_passage();
	
		};

	}
}


#endif // CAMERA_MANAGER_H