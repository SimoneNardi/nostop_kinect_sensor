#include "Robot.h"
#include "Robot_manager.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "Ball_tracker.h"
#include "Camera.h"
#include "math.h"
#include <string>

#include "Box.h"
#include <Quaternion.h>
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot::Robot(std::string name_):
  m_heading(0)
, found(false)
, m_notFoundCount(0)
, m_name(name_)
{ 
  m_front_marker_color = m_name.substr(0,m_name.find("_"));
  m_back_marker_color = m_name.substr(m_name.find("_")+1,m_name.length());
  m_robot_pub = m_robot.advertise<std_msgs::String>("/localizer/kinect/add_robot",1);
  m_robot_pose_pub = m_robot.advertise<geometry_msgs::Pose>("/"+m_name+"/localizer/camera/pose",1);
  Front_ptr = std::make_shared<Ball_tracker>();
  Back_ptr = std::make_shared<Ball_tracker>();
  ROS_INFO("ROBOT %s ON!",m_name.c_str());
}

Robot::~Robot()
{}


void Robot::pubID()
{
  std_msgs::String robot_name;
  robot_name.data = m_name.c_str();
  m_robot_pub.publish(robot_name);
}

 
void Robot::select_robot_pose(std::vector<ball_position>& front_array,std::vector<ball_position>& back_array)
{	
  float distance;
  if(!found)
  {
   for (size_t i = 0;i < front_array.size();i++)
    { 
     for(size_t j = 0;j < back_array.size();j++)
     {
	distance = sqrt(pow((front_array[i].x-back_array[j].x),2)+pow((front_array[i].y-back_array[j].y),2));
	if(distance < 2*(front_array[i].width+back_array[j].width))
	  {
	    m_front_pos = front_array[i];
	    m_back_pos = back_array[j]; 
	    found = true;
	    m_f_rect = Front_ptr->kalman_update(m_front_pos);
	    m_b_rect = Back_ptr->kalman_update(m_back_pos);
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
	    if(distance < 2*(front_array[i].width+back_array[j].width) && m_f_rect.contains(front) && m_b_rect.contains(back))
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
		      m_front_pos.x = m_f_rect.x;
		      m_front_pos.y = m_f_rect.y;
		      m_back_pos.x = m_b_rect.x;
		      m_back_pos.y = m_b_rect.y;
		      m_notFoundCount++;
		    }
	    }
	  }
	}
    }
  m_heading = atan2((m_back_pos.y-m_front_pos.y),(m_back_pos.x-m_front_pos.x))+M_PI;
  publish_pose(m_front_pos,m_back_pos,m_heading);
}



void Robot::publish_pose(ball_position front_pos,ball_position back_pos, float yaw)
{
    geometry_msgs::Pose pose;
    float phi = 0;//ROLL
    float theta = 0;//PITCH
    float psi = yaw;
    pose.position.x = (front_pos.x+back_pos.x)/2;
    pose.position.y = (front_pos.y+back_pos.y)/2;
    pose.position.z = 0; 
    pose.orientation.x = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2); 
    pose.orientation.y = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2); 
    pose.orientation.z = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
    pose.orientation.w = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
    m_robot_pose_pub.publish<geometry_msgs::Pose>(pose);
}


std::string Robot::color_f()
{
  std::string front;
  front.assign(m_front_marker_color);
  return front;
}

std::string Robot::color_b()
{
  std::string back;
  back.assign(m_back_marker_color);
  return back;
}



