#include <ros/ros.h>
#include "Fake_imu.h"


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
	
	
Fake_imu::Fake_imu(std::string& robot_name):
is_imu_readed(false)
,is_odom_readed(false)
{
	m_imu_sub = m_node.subscribe<sensor_msgs::Imu>("/"+robot_name+"/imu_data",10,&Fake_imu::imu_readed,this);
	m_odom_cam_sub = m_node.subscribe<nav_msgs::Odometry>("/"+robot_name+"/cam_odom",10,&Fake_imu::cam_odom_readed,this);
	m_fake_imu_pub = m_node.advertise<sensor_msgs::Imu>("/"+robot_name+"/fake_imu_data",10);
}

	
Fake_imu::~Fake_imu(){}



void Fake_imu::imu_readed(const sensor_msgs::Imu::ConstPtr& msg)
{
      if(!is_imu_readed)
      {
		Lock l_lock(m_mutex);
		m_imu_msg.angular_velocity = msg->angular_velocity;
		m_imu_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
		m_imu_msg.header = msg->header;
		m_imu_msg.linear_acceleration = msg->linear_acceleration;
		m_imu_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;
		is_imu_readed = true;
      }
}


void Fake_imu::cam_odom_readed(const nav_msgs::Odometry::ConstPtr& msg)
{
      if(!is_odom_readed)
      {
		Lock l_lock(m_mutex);
		m_imu_msg.orientation.x = msg->pose.pose.orientation.x;
		m_imu_msg.orientation.y = msg->pose.pose.orientation.y;
		m_imu_msg.orientation.z = msg->pose.pose.orientation.z;
		m_imu_msg.orientation.w = msg->pose.pose.orientation.w;
		for (int i = 0;i<9;++i)
		{
			m_imu_msg.orientation_covariance.at(i) = 0.1;
			i=i+3;
		}
		is_odom_readed = true;
      }
}

void Fake_imu::imu_published()
{
	if(is_imu_readed==true && is_odom_readed==true)
	{
		Lock l_lock(m_mutex);
		m_fake_imu_pub.publish<sensor_msgs::Imu>(m_imu_msg);
		is_imu_readed = false;
		is_odom_readed = false;
	}
}



