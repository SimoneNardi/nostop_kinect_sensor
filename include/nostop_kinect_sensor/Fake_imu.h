////////////////////////////////////////////////////////////
//	Fake_imu.h
//	Created on:	22-02-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef FAKEIMU_H
#define FAKEIMU_H
#pragma once

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Threads.h>
namespace Robotics 
{
	namespace GameTheory
	{
		
		class Fake_imu
		{
			ros::NodeHandle m_node;
			ros::Subscriber m_imu_sub;
			ros::Subscriber m_odom_cam_sub;
			ros::Publisher m_fake_imu_pub;
			sensor_msgs::Imu m_imu_msg;
			Mutex m_mutex;
			bool is_imu_readed,is_odom_readed;
		  
		public:
			Fake_imu(std::string& robot_name);
			void imu_readed(const sensor_msgs::Imu::ConstPtr& msg);
			void cam_odom_readed(const nav_msgs::Odometry::ConstPtr& msg);
			void imu_published();
			~Fake_imu();
		};

	}
}


#endif // FAKEIMU_H