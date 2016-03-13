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
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
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
		  ros::Publisher m_robot_heading_pub,m_robot_gps_pub;
		  ros::Subscriber m_robot_command;
		  std::string m_name;
		  std::string m_front_marker_color;
		  std::string m_back_marker_color;
		  double m_lat0,m_lon0;
		  sensor_msgs::NavSatFix m_initial_pose_gps;
		  Mutex m_mutex;
		  // MARKER POSITION
		  ball_position m_front_pos;
		  ball_position m_back_pos;
		  float m_heading;
		  bool found,m_before_cmd;
		  int m_notFoundCount;
		  cv::Rect m_f_rect;
		  cv::Rect m_b_rect;
		  std::shared_ptr<Ball_tracker> Front_ptr;
		  std::shared_ptr<Ball_tracker> Back_ptr;
		  tf::TransformBroadcaster m_static_transform_broadcaster;
		  geometry_msgs::TransformStamped m_static_tf;
		  
	public:
		  Robot(std::string& name, double& lat, double& lon);
		  sensor_msgs::NavSatFix enu2geodetic(double& x,double& y,double& z);
		  void publish_pose(ball_position front_pos,ball_position back_pos,float yaw);
		  void select_robot_pose(std::vector<ball_position>& front_array,std::vector<ball_position>& back_array);
		  void command_reading(const geometry_msgs::Twist::ConstPtr& msg);
		  std::string color_f();
		  std::string color_b();
		  void static_transform_publishing(double& x,double& y);
		    ~Robot();
		};

	}
}


#endif // ROBOT_H
