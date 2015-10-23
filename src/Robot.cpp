#include "Robot.h"
#include "Robot_manager.h"
#include "ros/ros.h"
#include "Tracker.h"
#include "nostop_kinect_sensor/Id_robot.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot::Robot()
{ 
  m_robot_pub = m_robot.advertise<nostop_kinect_sensor::Id_robot>("TO_DO",1);
  Front_ptr = std::make_shared<Tracker>();
  Back_ptr = std::make_shared<Tracker>();
}

Robot::~Robot()
{}


void Robot::pubID()
{
  nostop_kinect_sensor::Id_robot l_msgs;
  l_msgs.name = m_name;
  l_msgs.back_marker_color = m_back_marker_color;
  l_msgs.front_marker_color = m_front_marker_color;
  m_robot_pub.publish(l_msgs);
}

void Robot::takeImgFront(cv::Mat img)
{
 m_Front = img;
}

void Robot::takeImgBack(cv::Mat img)
{
 m_Back = img;
}

void Robot::pose_heading()
{
   Front_ptr->toGetPos(m_front_pos);
   Back_ptr->toGetPos(m_back_pos);
}