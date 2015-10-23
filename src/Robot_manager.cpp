#include "Robot_manager.h"
#include "Robot.h"
#include "Collection.h"
#include "ros/ros.h"
#include <string.h>

#include "nostop_kinect_sensor/Id_robot.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot_manager::Robot_manager(): 
m_robot_count(0)
{
  m_robot_in = m_manager_node.subscribe<nostop_kinect_sensor::Id_robot>(" ", 1, &Robot_manager::new_robot_id,this);
}


Robot_manager::~Robot_manager()	
{}

void Robot_manager::new_robot_id(const nostop_kinect_sensor::Id_robot::ConstPtr& msg)// INPUT??
{	
  m_robot_id_array.at(m_robot_count) = {msg->name,msg->front_marker_color,msg->back_marker_color};
  m_robot_ptr_array.at(m_robot_count) = std::make_shared<Robot>();
  m_robot_count++;
}


void Robot_manager::threshold_update(cv::Mat blue, cv::Mat green, cv::Mat red, cv::Mat yellow)
{
  for(int i=0;i<m_robot_count+1;i++)
  {
    robot_id = m_robot_id_array.at(i);
    m_robot_single_ptr = m_robot_ptr_array.at(i);
    // FRONT
    if(robot_id.front_marker_color == "blue")
    {
     m_robot_single_ptr->takeImgFront(blue);
    }
    if(robot_id.front_marker_color == "green")
    {
      m_robot_single_ptr->takeImgFront(green); 
    }
    if(robot_id.front_marker_color == "red")
    {
      m_robot_single_ptr->takeImgFront(red);
    }
    if(robot_id.front_marker_color == "yellow")
    {
      m_robot_single_ptr->takeImgFront(yellow);
    }
    
    // BACK
    if(robot_id.back_marker_color == "blue")
    {
     m_robot_single_ptr->takeImgBack(blue);
    }
    if(robot_id.back_marker_color == "green")
    {
      m_robot_single_ptr->takeImgBack(green); 
    }
    if(robot_id.back_marker_color == "red")
    {
      m_robot_single_ptr->takeImgBack(red);
    }
    if(robot_id.back_marker_color == "yellow")
    {
      m_robot_single_ptr->takeImgBack(yellow);
    }
    
    
    
  }
}