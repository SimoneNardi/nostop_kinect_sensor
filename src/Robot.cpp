#include "Robot.h"
#include "Robot_manager.h"
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include "Ball_tracker.h"
#include "Camera.h"
#include "math.h"
#include <string>
#include "ball_position.h"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////
Robot::Robot(std::string& name, double& lat0, double& lon0):
  m_heading(0)
, found(false)
, m_notFoundCount(0)
, m_name(name)
, m_front_pos()
, m_back_pos()
, m_lat0(lat0)
, m_lon0(lon0)
{ 
	m_front_marker_color = m_name.substr(0,m_name.find("_"));
	m_back_marker_color = m_name.substr(m_name.find("_")+1,m_name.length());
	m_robot_gps_pub = m_robot.advertise<sensor_msgs::NavSatFix>("/"+m_name+"/localizer/gps/fix",1);
	m_robot_heading_pub= m_robot.advertise<std_msgs::Float64>("/"+m_name+"/heading",1);
	Front_ptr = std::make_shared<Ball_tracker>();
	Back_ptr = std::make_shared<Ball_tracker>();
	ROS_INFO("ROBOT %s ON!", m_name.c_str());
}

//////////////////////////////////////
Robot::~Robot()
{}

//////////////////////////////////////
void Robot::select_robot_pose(std::vector<ball_position>& front_array,std::vector<ball_position>& back_array)
{	
	float distance;
	if(front_array.size() > 0 && back_array.size() > 0)// Robot found!
	{
		if(!found)
		{
			for (size_t i = 0;i < front_array.size();i++)
			{ 
				for(size_t j = 0;j < back_array.size();j++)
				{
					distance = sqrt(pow((front_array[i].x-back_array[j].x),2)+pow((front_array[i].y-back_array[j].y),2));
					if(distance < 2.5*(front_array[i].width+back_array[j].width))
					{
						m_front_pos = front_array[i];
						m_back_pos = back_array[j]; 
						found = true;
						m_f_rect = Front_ptr->kalman_update(m_front_pos);
						m_b_rect = Back_ptr->kalman_update(m_back_pos);
						m_heading = atan2((m_back_pos.y-m_front_pos.y),(m_back_pos.x-m_front_pos.x))+M_PI;
						publish_pose(m_front_pos,m_back_pos,m_heading);
					}else{
						found = false;
					}
				}
			}
		}else{
			for (size_t i = 0;i < front_array.size();i++)
			{ 
				for(size_t j = 0;j < back_array.size();j++)
				{
					cv::Point2f front,back;
					front.x = front_array[i].x;
					front.y = front_array[i].y;
					back.x = back_array[j].x;
					back.y = back_array[j].y;
					distance = sqrt(pow((front.x-back.x),2)+pow((front.y-back.y),2));
					if(distance < 2.5*(front_array[i].width+back_array[j].width)
					  && m_f_rect.contains(front) && m_b_rect.contains(back))
					{
						m_front_pos = front_array[i];
						m_back_pos = back_array[j]; 
						found = true;
						m_f_rect = Front_ptr->kalman_update(m_front_pos);
						m_b_rect = Back_ptr->kalman_update(m_back_pos);
					}else{
						if(m_notFoundCount >= 10 )
						{
							m_notFoundCount = 0;
							found = false;
						}else{
							m_f_rect = Front_ptr->kalman_update(m_front_pos);
							m_b_rect = Back_ptr->kalman_update(m_back_pos);
							m_front_pos.x = m_f_rect.x+m_f_rect.width/2;
							m_front_pos.y = m_f_rect.y+m_f_rect.width/2;
							m_back_pos.x = m_b_rect.x+m_f_rect.width/2;
							m_back_pos.y = m_b_rect.y+m_f_rect.width/2;
							m_notFoundCount++;
						}
					}
				}
			}
		m_heading = atan2((m_back_pos.y-m_front_pos.y),(m_back_pos.x-m_front_pos.x))+M_PI;
		publish_pose(m_front_pos,m_back_pos,m_heading);
		}
	}
}


sensor_msgs::NavSatFix Robot::enu2geodetic(double& x,double& y,double& z)
{
	sensor_msgs::NavSatFix GPS;
	GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),GeographicLib::Constants::WGS84_f());
	GeographicLib::LocalCartesian proj(m_lat0, m_lon0, 0, earth);
	double lat, lon, h;
	proj.Reverse(x, y, z, lat, lon, h);
	GPS.latitude = lat;
	GPS.longitude = lon;
	GPS.altitude = h;
	GPS.position_covariance.at(0) = 0.1;
	GPS.position_covariance.at(4) = 0.1;
	GPS.position_covariance.at(8) = 0.1;
	GPS.header.frame_id = m_name+"/base_link";// antenna location
	GPS.header.stamp = ros::Time::now();
	return GPS;
}



//////////////////////////////////////
void Robot::publish_pose(ball_position front_pos,ball_position back_pos, float yaw)
{
	// heading pub
	std_msgs::Float64 heading;
	heading.data = yaw;
	m_robot_heading_pub.publish<std_msgs::Float64>(heading);
	// gps pub
	sensor_msgs::NavSatFix pose_gps;
	double x = 0.01*(front_pos.x+back_pos.x)/2;
	double y = 0.01*(front_pos.y+back_pos.y)/2;
	double z = 0;
	pose_gps = enu2geodetic(x,y,z);// CORRECTION BECAUSE x NOT POINT TO EAST?
	m_robot_gps_pub.publish<sensor_msgs::NavSatFix>(pose_gps);          
}




//////////////////////////////////////
std::string Robot::color_f()
{
	std::string front;
	front.assign(m_front_marker_color);
	return front;
}

//////////////////////////////////////
std::string Robot::color_b()
{
	std::string back;
	back.assign(m_back_marker_color);
	return back;
}



