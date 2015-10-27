#include "Robot.h"
#include "Robot_manager.h"
#include "ros/ros.h"
#include "Tracker.h"
#include "nostop_agent/Id_robot.h"
#include "Collection.h"
#include "math.h"
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot::Robot(std::string & name_,std::string & front_color_,std::string & back_color_)
: m_name(name_)
, m_front_marker_color(front_color_)
, m_back_marker_color(back_color_)
{ 
  m_robot_pub = m_robot.advertise<nostop_agent::Id_robot>("/localizer/kinect/add_robot",1);
  Front_ptr = std::make_shared<Tracker>();
  Back_ptr = std::make_shared<Tracker>();
}

Robot::~Robot()
{}


void Robot::pubID()
{
  nostop_agent::Id_robot l_msgs;
  l_msgs.name = m_name;
  //l_msgs.back_marker_color = m_back_marker_color;
  //l_msgs.front_marker_color = m_front_marker_color;
  m_robot_pub.publish(l_msgs);
}



void Robot::select_robot_pose(std::vector<ball_position>& front_array,std::vector<ball_position>& back_array,cv::Mat src)
{	
//   float distance;
//    for (int i = 0;i < front_count+1;i++)
//    {
//      for(int j = 0;j < back_count+1;j++)
//      {
// 	distance = sqrt(pow((front_array[i].x-back_array[j].x),2)+pow((front_array[i].y-back_array[j].y),2));
//        if (distance < 2*(front_array[i].width+back_array[j].width))
//        {
// 	 m_front_pos = front_array[i];
// 	 m_back_pos = back_array[j];
// 	}
// 	
//     }
//   }
//   Front_ptr->kalman_update(m_front_pos);// USING PREDICTION?
//   Back_ptr->kalman_update(m_back_pos);
//   cv::Mat dst;
//   draw_circles(src);
//   m_heading = atan2((m_back_pos.y-m_front_pos.y),(m_back_pos.x-m_front_pos.x))-M_PI;
//   ROS_INFO("%f",m_heading*180/M_PI);
}


void Robot::draw_circles(cv::Mat src)
{
  cv::Rect Box_f,Box_b;
  cv::Mat l_fdst,l_bdst;
  Box_f.height = m_front_pos.height;
  Box_f.width = m_front_pos.width;
  Box_f.x = m_front_pos.x;
  Box_f.y = m_front_pos.y;
  Box_b.height = m_back_pos.height;
  Box_b.width = m_back_pos.width;
  Box_b.x = m_back_pos.x;
  Box_b.y = m_back_pos.y;
  cv::rectangle(src, Box_f, CV_RGB(0,255,0), 2); // THE FRONT BALL IS GREEN
  cv::rectangle(src,Box_b, CV_RGB(255,0,0),2); // THE BACK BALL IS RED
  imshow("test",src );
}
