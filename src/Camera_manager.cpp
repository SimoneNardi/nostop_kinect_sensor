#include "Camera_manager.h"
#include "Camera.h"
#include "std_msgs/String.h"
#include <Robot_manager.h>

#include "nostop_kinect_sensor/Camera_data.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
Camera_manager::Camera_manager()
{
  ROS_INFO("COLLECTOR ON!");
 m_camera_in = m_node.subscribe<nostop_kinect_sensor::Camera_data>("/camera_in", 1000, &Camera_manager::new_camera,this);
 m_manager = std::make_shared<Robot_manager>();
}

/////////////////////////////////////////////
Camera_manager::~Camera_manager()
{}


void Camera_manager::new_camera(const nostop_kinect_sensor::Camera_data::ConstPtr& msg)
{	
    std::string camera_name,image_topic,calibration_topic;
    std::vector<float> pos_camera;
    float pitch, omega,gamma,R;
    camera_name.assign( msg->name);
    image_topic.assign(msg->topic_name);
    calibration_topic.assign(msg->calibration_topic);
    m_camera_array.push_back( std::make_shared<Camera>(camera_name,image_topic,calibration_topic) );
    
}

void Camera_manager::pack_passage()
{
    for(size_t i = 0; i<m_camera_array.size();i++)
    {
        std::vector<ball_position> l_blue,l_green,l_red,l_yellow;
	l_blue = m_camera_array[i]->get_blue_array();
	l_green = m_camera_array[i]->get_green_array();
	l_red = m_camera_array[i]->get_red_array();
	l_yellow = m_camera_array[i]->get_yellow_array();
	for(size_t j=0;j<l_blue.size();j++)
	  {
	    m_blue_ball_W.push_back(l_blue[j]);
	  }
      for(size_t j=0;j<l_green.size();j++)
	{
	  m_green_ball_W.push_back(l_green[j]);
	}
      for(size_t j=0;j<l_red.size();j++)
	{
	  m_red_ball_W.push_back(l_red[j]);
	}
      for(size_t j=0;j<l_yellow.size();j++)
	{
	  m_yellow_ball_W.push_back(l_yellow[j]);
	}
    }
    
    // TO ROBOT MANAGER
    Lock l_lock(m_mutex);
    m_manager->array_assignment(m_blue_ball_W,m_green_ball_W,m_red_ball_W,m_yellow_ball_W);
  // CLEARING
    m_blue_ball_W.clear();
    m_green_ball_W.clear();
    m_red_ball_W.clear();
    m_yellow_ball_W.clear();
  }
