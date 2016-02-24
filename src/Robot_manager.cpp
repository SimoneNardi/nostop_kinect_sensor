#include "Robot.h"
#include "Camera.h"
#include "ros/ros.h"
#include <string.h>
#include <boost/signals2/shared_connection_block.hpp>
#include "Robot_manager.h"
#include "nostop_agent/AddRobot.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////
Robot_manager::Robot_manager(double& lat0,double& lon0):
m_lat0(lat0)
, m_lon0(lon0)
{
  ROS_INFO("ROBOT MANAGER ON!");
  m_add_robot_topic = m_manager_node.subscribe<std_msgs::String>("/localizer/kinect/add_robot", 10, &Robot_manager::new_robot_id_topic,this);
  m_add_robot_service = m_manager_node.advertiseService("/localizer/add_robot", &Robot_manager::new_robot_id_service,this);
}

//////////////////////////////////////
Robot_manager::~Robot_manager()
{}

//////////////////////////////////////
bool Robot_manager::new_robot_id_service(
  nostop_agent::AddRobot::Request  &req,
  nostop_agent::AddRobot::Response &res)
{
  m_robot_array.push_back( std::make_shared<Robot>(req.name,m_lat0,m_lon0) );
  return true;
}

//////////////////////////////////////
void Robot_manager::new_robot_id_topic(const std_msgs::String::ConstPtr& msg)
{	
    std::string l_robot_name = msg->data;
    std::size_t found = l_robot_name.find_last_of("_");
    l_robot_name = l_robot_name.substr(0,found);
    m_robot_array.push_back( std::make_shared<Robot>(l_robot_name,m_lat0,m_lon0) );
}

//////////////////////////////////////
void Robot_manager::array_assignment(
  std::vector<ball_position>& blue_array, 
  std::vector<ball_position>& green_array,
  std::vector<ball_position>& red_array,
  std::vector<ball_position>& yellow_array)
{
  if (m_robot_array.size() != 0 )
  {
    for(size_t i=0;i<m_robot_array.size();i++)
    {
      std::vector<ball_position> l_front_array;
      std::vector<ball_position> l_back_array;

      if (m_robot_array[i]->color_f()== "blue")
	{
	  l_front_array = blue_array;
	}
      if (m_robot_array[i]->color_f() == "green")
	{
	  l_front_array = green_array;
	}
      if (m_robot_array[i]->color_f() == "red")
	{
	  l_front_array = red_array;
	}
      if (m_robot_array[i]->color_f() == "yellow")
	{
	  l_front_array = yellow_array;
	}
            
      if (m_robot_array[i]->color_b() == "blue")
	{
	  l_back_array = blue_array;
	}
      if (m_robot_array[i]->color_b() == "green")
	{
	  l_back_array = green_array;
	}
      if (m_robot_array[i]->color_b() == "red")
	{
	  l_back_array = red_array;
	}
      if (m_robot_array[i]->color_b() == "yellow")
	{
	  l_back_array = yellow_array;
	}
      
      m_robot_array[i]->select_robot_pose(l_front_array,l_back_array);

    }
  }
}

