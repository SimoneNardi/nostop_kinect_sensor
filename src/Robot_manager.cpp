#include "Robot.h"
#include "Collection.h"
#include "ros/ros.h"
#include <string.h>
#include <boost/signals2/shared_connection_block.hpp>
#include "Robot_manager.h"
#include "nostop_agent/Id_robot.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot_manager::Robot_manager(){
  m_robot_in = m_manager_node.subscribe<nostop_agent::Id_robot>("/localizer/kinect/add_robot", 1000, &Robot_manager::new_robot_id,this);
  }


Robot_manager::~Robot_manager()	
{}


void Robot_manager::new_robot_id(const nostop_agent::Id_robot::ConstPtr& msg)
{	
    m_robot_array.push_back( std::make_shared<Robot>(msg->name) );
    ROS_INFO("New ROBOT!");
}

void Robot_manager::array_assignment(
  std::vector<ball_position>& blue_array, 
  std::vector<ball_position>& green_array,
  std::vector<ball_position>& red_array,
  std::vector<ball_position>& yellow_array)
{
  if (m_robot_array.size() == 0 )
  {}
  else{
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

