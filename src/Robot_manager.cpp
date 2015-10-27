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
{}


Robot_manager::~Robot_manager()	
{}


void Robot_manager::subscribe()
{
   m_robot_in = m_manager_node.subscribe<nostop_kinect_sensor::Id_robot>("/robot_id", 1000, &Robot_manager::new_robot_id,this);
}

void Robot_manager::new_robot_id(const nostop_kinect_sensor::Id_robot::ConstPtr& msg)
{	
   ROS_INFO("%d", m_robot_count);
   m_robot_id_array[m_robot_count].name = msg->name;
   m_robot_id_array[m_robot_count].front_marker_color = msg->front_marker_color;
   m_robot_id_array[m_robot_count].back_marker_color = msg->back_marker_color;
   ROS_INFO("New ROBOT!");
   m_robot_count++;
}


void Robot_manager::array_assignment(ball_position blue_array[], ball_position green_array[], ball_position red_array[], ball_position yellow_array[], int blue_count, int green_count, int red_count, int yellow_count,cv::Mat stream)
{
    for(int i=0;i<m_robot_count+1;i++)
    {
      ball_position * l_front_array = new ball_position[ROBOT_NUMBER];
      int l_front_count;
      robot_id = m_robot_id_array[i];
    if (robot_id.front_marker_color == "blue")
    {
      l_front_array = blue_array;
      l_front_count = blue_count;
    }
    if (robot_id.front_marker_color == "green")
    {
      l_front_array = green_array;
      l_front_count = green_count;
    }
    if (robot_id.front_marker_color == "red")
    {
      l_front_array = red_array;
      l_front_count = red_count;
    }
    if (robot_id.front_marker_color == "yellow")
    {
      l_front_array = yellow_array;
      l_front_count = yellow_count;
    }
    if (robot_id.back_marker_color == "blue")
    {
      m_robot_single_ptr->select_robot_pose(l_front_array,blue_array,l_front_count,blue_count,stream);
    }
    if (robot_id.back_marker_color == "green")
    {
      m_robot_single_ptr->select_robot_pose(l_front_array,green_array,l_front_count,green_count,stream);
    }
    if (robot_id.back_marker_color == "red")
    {
      m_robot_single_ptr->select_robot_pose(l_front_array,red_array,l_front_count,red_count,stream);
    }
    if (robot_id.back_marker_color == "yellow")
    {
      m_robot_single_ptr->select_robot_pose(l_front_array,yellow_array,l_front_count,yellow_count,stream);
    }
    }
}

