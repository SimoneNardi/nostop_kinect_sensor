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

		
		typedef struct CallbackMNGI
		{
		  std::string cam_name;
		  std::string robot_name;
		  bool head_callback_done,tail_callback_done;
		}CallbackMNG;
		
		class Camera;
		
		class Camera_manager
		{
			Mutex m_mutex;

			ros::Publisher m_pub;
			ros::Subscriber m_camera_in;
			ros::Subscriber m_add_robot_topic;
			std::vector< std::shared_ptr<Camera> > m_camera_array;
			std::vector<std::string> m_camera_names;
			std::shared_ptr<Robot_manager> m_manager;
			
			// ROBOT
			std::vector<RobotConfiguration*> m_robot_initial_configuration;
			std::vector<CameraImgName> m_camera_on;
			std::vector<CallbackMNG> m_callback;
			std::set< MouseCallbackData*, less_MouseCallbackData > m_initialization_data;
			
			ros::NodeHandle m_node;
		
		public:
			Camera_manager(double& lat,double& lon);
			~Camera_manager();
			void new_robot_id_topic(const std_msgs::String::ConstPtr& msg);
			void initialize_mouse();
			void new_camera(const nostop_kinect_sensor::Camera_data::ConstPtr& msg);
 			void pack_passage();
		};

	}
}


#endif // CAMERA_MANAGER_H