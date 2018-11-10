#include "Robot.h"
#include "Robot_manager.h"
#include "ros/ros.h"
#include "Tracker.h"
#include "nostop_agent/Id_robot.h"
#include "Collection.h"
#include "math.h"
#include <string>

#include "Box.h"
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot::Robot(std::string name_):
 m_heading(0)
,found(false)
, m_notFoundCount(0)
{ 
  m_name = name_;
  m_front_marker_color = m_name.substr(0,m_name.find("_"));
  m_back_marker_color = m_name.substr(m_name.find("_")+1,m_name.length());
  m_robot_pub = m_robot.advertise<nostop_agent::Id_robot>("/localizer/kinect/add_robot",1);
  Front_ptr = std::make_shared<Tracker>();
  Back_ptr = std::make_shared<Tracker>();
  ROS_INFO("ROBOT %s ON!",m_name.c_str());
}

Robot::~Robot()
{}


void Robot::pubID()
{
  nostop_agent::Id_robot l_msgs;
  l_msgs.name = m_name;
  m_robot_pub.publish(l_msgs);
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
//      ROS_INFO("x-->%f",m_front_pos.x);
//        ROS_INFO("y-->%f",m_front_pos.y);
//   draw_circles(src);
  m_heading = atan2((m_back_pos.y-m_front_pos.y),(m_back_pos.x-m_front_pos.x));
//   ROS_INFO("%f",m_heading);
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




// void Robot::draw_circles(cv::Mat src)
// {
//   cv::Rect Box_f,Box_b;
//   cv::Mat l_fdst,l_bdst;
//   Box_f.height = m_front_pos.height;
//   Box_f.width = m_front_pos.width;
//   Box_f.x = m_front_pos.x;
//   Box_f.y = m_front_pos.y;
//   Box_b.height = m_back_pos.height;
//   Box_b.width = m_back_pos.width;
//   Box_b.x = m_back_pos.x;
//   Box_b.y = m_back_pos.y;
//   cv::rectangle(src, Box_f, CV_RGB(255,0,0), 5); // THE FRONT BALL IS IN RED BOX
//   cv::rectangle(src,Box_b, CV_RGB(0,255,0),5); // THE BACK BALL IS IN GREEN BOX
//   cv::rectangle(src,m_f_rect, CV_RGB(0,255,255),2); // THE BACK BALL IS IN TURQUOISE BOX
//   cv::rectangle(src,m_b_rect, CV_RGB(255,0,255),2); // THE BACK BALL IS IN FUCHSIA BOX
//   imshow("robot "+ m_name,src );
// }

