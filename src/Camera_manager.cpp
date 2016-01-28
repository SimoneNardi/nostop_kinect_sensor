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
	m_add_robot_topic = m_node.subscribe<std_msgs::String>("/localizer/kinect/add_robot", 1000, &Camera_manager::new_robot_id_topic,this);
	m_manager = std::make_shared<Robot_manager>();
}

/////////////////////////////////////////////
Camera_manager::~Camera_manager()
{}

/////////////////////////////////////////////
void Camera_manager::new_camera(const nostop_kinect_sensor::Camera_data::ConstPtr& msg)
{	
	std::string camera_name,image_topic,calibration_topic;
	camera_name.assign( msg->name);
	image_topic.assign(msg->topic_name);
	calibration_topic.assign(msg->calibration_topic);
	float ifovx,ifovy;
	ifovx = msg->ifovx;
	ifovy = msg->ifovy;
	m_camera_array.push_back( std::make_shared<Camera>(camera_name,image_topic,calibration_topic,ifovx,ifovy) );
	CameraImgName camera;
	camera.camera_name =  camera_name;
	m_camera_on.push_back(camera);
}

void Camera_manager::new_robot_id_topic(const std_msgs::String::ConstPtr& msg)
{
	RobotConfiguration l_robot;
	l_robot.name = msg->data;
	l_robot.pose_setted = -1;
	m_robot_initial_configuration.push_back(l_robot);
}

//Mouse Callbacks
void mouse_callback_head_point(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) 
	{
		RobotConfiguration *l_robot = (RobotConfiguration*) param;
		l_robot->head_point.x = x;
		l_robot->head_point.y = y;
		l_robot->pose_setted = 0;
	}
}

void mouse_callback_tail_point(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) 
	{
		RobotConfiguration *l_robot = (RobotConfiguration*) param;
		cv::Rect pose;
		l_robot->tail_point.x = x;
		l_robot->tail_point.y = y;
		l_robot->odom_SR_origin_pix.x = (l_robot->head_point.x+l_robot->tail_point.x)/2;
		l_robot->odom_SR_origin_pix.y = (l_robot->head_point.y+l_robot->tail_point.y)/2;
		pose.height = 200;//TODO
		pose.width = 200;
		pose.x = l_robot->odom_SR_origin_pix.x-pose.width/2;
		pose.y = l_robot->odom_SR_origin_pix.y-pose.height/2;
		l_robot->pose_rect = pose;
		l_robot->pose_setted = 1;
	}
}


// SETTING INITIAL POSE
void Camera_manager::initialize_mouse() // CASE AGAINST IF?
{ 
  // ROBOTS INITIAL POSE
	for(size_t i = 0; i<m_robot_initial_configuration.size(); ++i)
	{
		for(size_t j = 0;j<m_camera_on.size();j++)
		{
			ball_position SR_cm,SR_pix;
			switch (m_robot_initial_configuration[i].pose_setted)
			{
				case -1:
				{
					m_camera_on[j].image = m_camera_array.at(j)->get_stream_video();
					std::string windows_name = m_robot_initial_configuration[i].name + " " + m_camera_on[j].camera_name  + " initial_pose";
					char* mouse_windows_name = new char[windows_name.size() + 1];
					std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
					mouse_windows_name[windows_name.size()] = '\0'; 
					cv::imshow(windows_name,m_camera_on[j].image );
					cvSetMouseCallback(mouse_windows_name,mouse_callback_head_point, &(m_robot_initial_configuration[i]) );
					break;
				}  
					
				case 0:
				{
					m_camera_on[j].image = m_camera_array.at(j)->get_stream_video();
					std::string windows_name = m_robot_initial_configuration[i].name + " " + m_camera_on[j].camera_name  + " initial_pose";
					char* mouse_windows_name = new char[windows_name.size() + 1];
					std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
					mouse_windows_name[windows_name.size()] = '\0'; 
					cv::imshow(windows_name,m_camera_on[j].image );
					cvSetMouseCallback(mouse_windows_name,mouse_callback_tail_point, &(m_robot_initial_configuration[i]) );
					break;
				}
					
				case 1:
				{
					std::string windows_name = m_robot_initial_configuration[i].name + " " + m_camera_on[j].camera_name + " initial_pose";
					char* mouse_windows_name = new char[windows_name.size() + 1];
					std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
					mouse_windows_name[windows_name.size()] = '\0'; 
					cvDestroyWindow(mouse_windows_name);
					m_robot_initial_configuration[i].pose_setted = 2;
					break;
				}
					
					
				case 2://// IT'S TRUE? TODO
					if(m_camera_array.size() == 1 ) 
					{	
						SR_pix.x = m_robot_initial_configuration[i].odom_SR_origin_pix.x;
						SR_pix.y = m_robot_initial_configuration[i].odom_SR_origin_pix.y;
						SR_cm = m_camera_array.at(0)->origin_pix2origin_world(SR_pix);
						m_robot_initial_configuration[i].odom_SR_origin_cm.x = SR_cm.x;
						m_robot_initial_configuration[i].odom_SR_origin_cm.y = SR_cm.y;
						m_robot_initial_configuration[i].pose_setted = 3;
					}else{
						m_camera_array.at(j-1)->origin_pix2origin_world(SR_pix);
						m_robot_initial_configuration[i].odom_SR_origin_cm.x = SR_cm.x;
						m_robot_initial_configuration[i].odom_SR_origin_cm.y = SR_cm.y;
						m_robot_initial_configuration[i].pose_setted = 3;
					}
					
					
				case 3:
				{	
					m_camera_array.at(j)->robot_topic_pose_subscribe(m_robot_initial_configuration[i]);
					m_robot_initial_configuration[i].pose_setted = 3;
					break;
				}  
					
				case 4:
				{
					//nothing to do 
					break;
				}
					
				default:
				{
					ROS_ERROR("Some errors. :-(");
					break;
				}
			}
		}
	} 
}   
    
  
/////////////////////////////////////////////
void Camera_manager::pack_passage()
{
	initialize_mouse();
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
