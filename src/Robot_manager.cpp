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
//    m_robot_ptr_array[m_robot_count]->insert(msg->name,msg->front_marker_color,msg->back_marker_color);
//    m_robot_ptr_array[m_robot_count] = std::make_shared<Robot>();
//    m_robot_ptr_array[m_robot_count]->insert(m_robot_id_array[m_robot_count].name,m_robot_id_array[m_robot_count].front_marker_color,m_robot_id_array[m_robot_count].back_marker_color);
   ROS_INFO("SI");
//    m_robot_ptr_array[0]=std::make_shared<Robot>();
  m_robot_count++;
//   m_robot_id_array[0].
 
  
}


void Robot_manager::threshold_update(cv::Mat blue, cv::Mat green, cv::Mat red, cv::Mat yellow, cv::Mat blue_circles_out, cv::Mat green_circles_out, cv::Mat red_circles_out, cv::Mat yellow_circles_out)
{
  if (m_robot_count==0)
  {}else
  {
    cv::Mat l_blue_circles,l_green_circles,l_red_circles,l_yellow_circles;
    l_blue_circles=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     l_green_circles=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     l_red_circles=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     l_yellow_circles=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     blue_circles_out=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     green_circles_out=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     red_circles_out=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
     yellow_circles_out=cv::Mat::zeros(blue.rows,blue.cols, blue.type());
    for(int i=0;i<m_robot_count+1;i++)
    {
    robot_id = m_robot_id_array[i];
//     m_robot_single_ptr = m_robot_ptr_array[i];
    // FRONT
    if(robot_id.front_marker_color == "blue")
    {
      m_robot_single_ptr->frontCircles(blue,l_blue_circles);
     cv::addWeighted(blue_circles_out,1.0,l_blue_circles,1.0,0.0,blue_circles_out);
    }
    if(robot_id.front_marker_color == "green")
    {
      m_robot_single_ptr->frontCircles(green,l_green_circles); 
      cv::addWeighted(green_circles_out,1.0,l_green_circles,1.0,0.0,green_circles_out);
    }
    if(robot_id.front_marker_color == "red")
    {
      m_robot_single_ptr->frontCircles(red,l_red_circles);
      cv::addWeighted(red_circles_out,1.0,l_red_circles,1.0,0.0,red_circles_out);

    }
    if(robot_id.front_marker_color == "yellow")
    {
      m_robot_single_ptr->frontCircles(yellow,l_yellow_circles);
      cv::addWeighted(yellow_circles_out,1.0,l_yellow_circles,1.0,0.0,yellow_circles_out);
    }
    
    // BACK
    if(robot_id.back_marker_color == "blue")
    {
     m_robot_single_ptr->backCircles(blue,l_blue_circles);
     cv::addWeighted(blue_circles_out,1.0,l_blue_circles,1.0,0.0,blue_circles_out);
    }
    if(robot_id.back_marker_color == "green")
    {
      m_robot_single_ptr->backCircles(green,l_green_circles); 
      cv::addWeighted(green_circles_out,1.0,l_green_circles,1.0,0.0,green_circles_out);

    }
    if(robot_id.back_marker_color == "red")
    {
      m_robot_single_ptr->backCircles(red,l_red_circles);
      cv::addWeighted(red_circles_out,1.0,l_red_circles,1.0,0.0,red_circles_out);
    }
    if(robot_id.back_marker_color == "yellow")
    {
      m_robot_single_ptr->backCircles(yellow,l_yellow_circles);
      cv::addWeighted(yellow_circles_out,1.0,l_yellow_circles,1.0,0.0,yellow_circles_out);
    }
    
    
    
    }
  }
}