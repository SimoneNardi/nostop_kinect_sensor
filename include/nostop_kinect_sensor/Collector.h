////////////////////////////////////////////////////////////
//	Collector.h
//	Created on:	07-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef COLLECTOR_H
#define COLLECTOR_H
#pragma once

// #include "ThreadBase.h"

#include "ros/ros.h"
#include <std_msgs/String.h>

#include <memory>

// #include "Threads.h"
#include "nostop_kinect_sensor/Camera_data.h"
#include "Collection.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class Collection;
		
		class Collector//: public ThreadBase	  	
		{

		  mutable Mutex m_mutex;

		  ros::Publisher m_pub;
		  ros::Subscriber m_camera_in;
		  std::vector< std::shared_ptr<Collection> > m_camera_array;
		  std::shared_ptr<Robot_manager> m_manager;
		  
		  // PACKAGE
		  std::vector<ball_position> m_blue_ball_W;
		  std::vector<ball_position> m_green_ball_W;
		  std::vector<ball_position> m_red_ball_W;
		  std::vector<ball_position> m_yellow_ball_W;

		  ros::NodeHandle m_node;
		protected:
		  //virtual void run();
		public:
			Collector();
			~Collector();
			void new_camera(const nostop_kinect_sensor::Camera_data::ConstPtr& msg);
 			void pack_passage();
	
		};

	}
}


#endif // COLLECTOR_H