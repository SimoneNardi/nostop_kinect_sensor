#include "Robot_manager.h"
#include "Robot.h"
#include "Collection.h"
#include "ros/ros.h"
#include <string.h>
#include <boost/signals2/shared_connection_block.hpp>

#include "nostop_agent/Id_robot.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot_manager::Robot_manager(){}


Robot_manager::~Robot_manager()	
{}


void Robot_manager::subscribe()
{
   m_robot_in = m_manager_node.subscribe<nostop_agent::Id_robot>("/localizer/kinect/add_robot", 1000, &Robot_manager::new_robot_id,this);
}

void Robot_manager::new_robot_id(const nostop_agent::Id_robot::ConstPtr& msg)
{	
   
   std::string test,l_front,l_back;
   test = msg->name;
   test.find("_");
   l_front = test.substr(0,test.find("_"));
   l_back = test.substr(test.find("_",test.length()));
   m_robot_array.push_back( std::make_shared<Robot>(msg->name) );
   
//    m_robot_id_array[m_robot_count].name = msg->name;
//    
//    // TODO
//    
//    m_robot_id_array[m_robot_count].front_marker_color = "blue";//msg->front_marker_color; // TODO
//    m_robot_id_array[m_robot_count].back_marker_color = "red";//msg->back_marker_color;
   ROS_INFO("New ROBOT!");
}

// void Robot_manager::array_assignment(
//   std::vector<ball_position>& blue_array, 
//   std::vector<ball_position>& green_array,
//   std::vector<ball_position>& red_array,
//   std::vector<ball_position>& yellow_array,cv::Mat stream)
// {
//     for(size_t i=0;i<m_robot_array.size();++i)
//     {
// //       std::vector<ball_position> l_front_array;
// //       std::vector<ball_position> l_back_array;
// //       
// //       if (m_robot_array[i]. == "blue")
// //       {
// // 	l_front_array = blue_array;
// //       }
// //       else if (m_robot_array[i].front_marker_color == "green")
// //       {
// // 	l_front_array = green_array;
// //       }
// //       else if (m_robot_array[i].front_marker_color == "red")
// //       {
// // 	l_front_array = red_array;
// //       }
// //       else if (m_robot_array[i].front_marker_color == "yellow")
// //       {
// // 	l_front_array = yellow_array;
// //       }
// //       
// //       if (m_robot_array[i].back_marker_color == "blue")
// //       {
// // 	l_back_array = blue_array;
// //       }
// //       if (m_robot_array[i].back_marker_color == "green")
// //       {
// // 	l_back_array = green_array;
// //       }
// //       if (m_robot_array[i].back_marker_color == "red")
// //       {
// // 	l_back_array = red_array;
// //       }
// //       if (m_robot_array[i].back_marker_color == "yellow")
// //       {
// // 	l_back_array = yellow_array;
// //       }
// //       
// //       m_robot_array[i]->select_robot_pose(l_front_array,l_back_array,stream);
// //       
//   }
// }
// 
