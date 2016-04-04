#include "Camera_manager.h"
#include "Camera.h"
#include "std_msgs/String.h"
#include <Robot_manager.h>

#include "nostop_kinect_sensor/Camera_data.h"


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
Camera_manager::Camera_manager(double& lat0,double& lon0)
{
	ROS_INFO("COLLECTOR ON!");
	m_camera_in = m_node.subscribe<nostop_kinect_sensor::Camera_data>("/camera_in", 10, &Camera_manager::new_camera,this);
	m_add_robot_topic = m_node.subscribe<std_msgs::String>("/localizer/camera/add_robot", 10, &Camera_manager::new_robot_id_topic,this);
	m_manager = std::make_shared<Robot_manager>(lat0,lon0);
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
	camera.waiting_for_head_click = false;
	camera.waiting_for_tail_click = false;
	m_camera_on.push_back(camera);
}

void Camera_manager::new_robot_id_topic(const std_msgs::String::ConstPtr& msg)
{
	RobotConfiguration *l_robot = new RobotConfiguration();
	l_robot->name =  msg->data;
	l_robot->is_magnetometer = false;
	if(l_robot->name.substr(l_robot->name.find_last_of("_")+1,l_robot->name.size()) == "ON")
	{
		l_robot->is_magnetometer = true;
		l_robot->name = l_robot->name.substr(0,l_robot->name.find_last_of("_"));
		ROS_INFO("Robot %s has magnetometer!",l_robot->name.c_str());
	}else{
		l_robot->name = l_robot->name.substr(0,l_robot->name.find_last_of("_"));
		ROS_INFO("Robot %s hasn't magnetometer!",l_robot->name.c_str());
	}
	l_robot->pose_setted = -1;
	
	m_robot_initial_configuration.push_back(l_robot);
}


//Mouse Callbacks
void mouse_callback_head_point(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) 
	{
		MouseCallbackData *l_data = (MouseCallbackData*) param;
		l_data->robot_config->head_point.x = x;
		l_data->robot_config->head_point.y = y;
		l_data->robot_config->pose_setted = 0;
		l_data->robot_config->cam_name = l_data->cam_name;
	}
}

void mouse_callback_tail_point(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) 
	{
		MouseCallbackData *l_data = (MouseCallbackData*) param;
		cv::Rect pose;
		l_data->robot_config->tail_point.x = x;
		l_data->robot_config->tail_point.y = y;
		l_data->robot_config->odom_SR_origin_pix.x = (l_data->robot_config->head_point.x+l_data->robot_config->tail_point.x)/2;
		l_data->robot_config->odom_SR_origin_pix.y = (l_data->robot_config->head_point.y+l_data->robot_config->tail_point.y)/2;
		pose.height = 200;
		pose.width = 200;
		pose.x = l_data->robot_config->odom_SR_origin_pix.x-pose.width/2;
		pose.y = l_data->robot_config->odom_SR_origin_pix.y-pose.height/2;
		l_data->robot_config->pose_rect = pose;
		l_data->robot_config->pose_setted = 1;
	}
}

void mouse_callback_central_point(int event, int x, int y, int flags, void* param)
{

	if (event == CV_EVENT_LBUTTONDBLCLK) 
	{
		
		MouseCallbackData *l_data = (MouseCallbackData*) param;
		cv::Rect pose;
		l_data->robot_config->central_point.x = x;
		l_data->robot_config->central_point.y = y;
		l_data->robot_config->odom_SR_origin_pix.x = l_data->robot_config->central_point.x;
		l_data->robot_config->odom_SR_origin_pix.y = l_data->robot_config->central_point.y;
		pose.height = 200;
		pose.width = 200;
		pose.x = l_data->robot_config->odom_SR_origin_pix.x-pose.width/2;
		pose.y = l_data->robot_config->odom_SR_origin_pix.y-pose.height/2;
		l_data->robot_config->pose_rect = pose;
		l_data->robot_config->pose_setted = 1;
		l_data->robot_config->cam_name = l_data->cam_name;
	}
}

// SETTING INITIAL POSE
void Camera_manager::initialize_mouse()
{ 
	// ROBOTS INITIAL POSE 
	for(size_t j = 0;j<m_camera_on.size();j++)
	{
		for(size_t i = 0; i<m_robot_initial_configuration.size(); i++)
		{
			ball_position SR_cm,SR_pix;
			
			std::string windows_name = m_robot_initial_configuration[i]->name + " " + m_camera_on[j].camera_name;
			
			switch ( m_robot_initial_configuration[i]->pose_setted )
			{
				case -1:
				{	
					m_camera_on[j].image = m_camera_array.at(j)->get_stream_video();
					if(m_camera_on[j].image.cols && m_camera_on[j].image.rows)
					{
						cv::imshow( windows_name.c_str(), m_camera_on[j].image );
						MouseCallbackData *l_callData=new MouseCallbackData;
						l_callData->cam_name = m_camera_on[j].camera_name;
						l_callData->robot_config = m_robot_initial_configuration[i];
						if(m_robot_initial_configuration[i]->is_magnetometer)
						{
							l_callData->type = MouseCallbackData::CENTRAL;
							
							if(m_initialization_data.find(l_callData) == m_initialization_data.end())
							{
								m_initialization_data.insert(l_callData);
								cvSetMouseCallback(windows_name.c_str(), mouse_callback_central_point, l_callData);
							}
							else
								delete l_callData;
						}
						else
						{
							l_callData->type = MouseCallbackData::HEAD;
							if(m_initialization_data.find(l_callData) == m_initialization_data.end())
							{
								m_initialization_data.insert(l_callData);
								cvSetMouseCallback(windows_name.c_str(), mouse_callback_head_point, l_callData);
							}
							else
							      delete l_callData;
						}
					}
					break;
				}  
					
				case 0:
				{
					m_camera_on[j].image = m_camera_array.at(j)->get_stream_video();
					cv::imshow(windows_name.c_str(), m_camera_on[j].image );
					
					MouseCallbackData *l_callData=new MouseCallbackData;
					l_callData->cam_name = m_camera_on[j].camera_name;
					l_callData->robot_config = m_robot_initial_configuration[i];
					l_callData->type = MouseCallbackData::TAIL;
					
					if(m_initialization_data.find(l_callData) == m_initialization_data.end())
					{
					  m_initialization_data.insert(l_callData);
					  cvSetMouseCallback(windows_name.c_str(), mouse_callback_tail_point, l_callData);
					}
					else
					  delete l_callData;
					
					break;
				}
					
				case 1:
				{
				    for(auto k= 0; k < m_camera_on.size(); ++k)
				    {
					std::string windows_name_local =  m_robot_initial_configuration[i]->name + " " + m_camera_on[k].camera_name;
					cvDestroyWindow( windows_name_local.c_str() );
				    }
					
				    m_robot_initial_configuration[i]->pose_setted = 2;
				    break;
				}
					
					
				case 2:
				{
					m_robot_initial_configuration[i]->pose_setted = 3;
					for(size_t k = 0; k < m_camera_array.size();++k)
					{
					  m_camera_array.at(k)->robot_topic_pose_subscribe( *(m_robot_initial_configuration[i]) );
					}
					break;
				}
				
					
				case 3:
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
	std::vector<ball_position> l_blue_ball_W,l_green_ball_W,l_red_ball_W,l_yellow_ball_W;
	for(size_t i = 0; i<m_camera_array.size();i++)
	{
		std::vector<ball_position> l_blue,l_green,l_red,l_yellow;
		l_blue = m_camera_array[i]->get_blue_array();
		l_green = m_camera_array[i]->get_green_array();
		l_red = m_camera_array[i]->get_red_array();
		l_yellow = m_camera_array[i]->get_yellow_array();
		for(size_t j=0;j<l_blue.size();j++)
		{
			l_blue_ball_W.push_back(l_blue[j]);
		}
		for(size_t j=0;j<l_green.size();j++)
		{
			l_green_ball_W.push_back(l_green[j]);
		}
		for(size_t j=0;j<l_red.size();j++)
		{
			l_red_ball_W.push_back(l_red[j]);
		}
		for(size_t j=0;j<l_yellow.size();j++)
		{
			l_yellow_ball_W.push_back(l_yellow[j]);
		}
	}
	// TO ROBOT MANAGER
	m_manager->array_assignment(l_blue_ball_W,l_green_ball_W,l_red_ball_W,l_yellow_ball_W);
  }
