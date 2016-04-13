////////////////////////////////////////////////////////////
//	Camera.h
//	Created on:	07-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef CAMERA_H
#define CAMERA_H
#pragma once

#include <map>

#include "Threads.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <opencv2/core/core.hpp>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "ros/ros.h"
#include "Robot_manager.h"
#include <nostop_kinect_sensor/Camera_data.h>
#include "ball_position.h"
#include "nostop_kinect_sensor/Camera_calibrationConfig.h"

namespace Robotics 
{
	namespace GameTheory
	{	  
		  class CameraID
		  {
		  public:
		    std::string name;
		    int number;
		  public:
		    CameraID(){}
		  };
		  
		  
		class RobotConfiguration
		{
		public:
			int pose_setted;
			long int gps_time;
			cv::Point2f head_point;
			cv::Point2f tail_point;
			cv::Point2f central_point;
			cv::Point2f odom_SR_origin_pix;
			cv::Rect pose_rect;
			bool is_magnetometer;
			std::string name;
			std::string cam_name;
			std::string front_color;
			std::string back_color;
			bool waiting_for_head_click,waiting_for_tail_click;
			public:
			      RobotConfiguration() {}
		};
			  
		  typedef struct CameraImgNameI
		  {
		    cv::Mat image;
		    std::string camera_name;
		    bool waiting_for_head_click, waiting_for_tail_click;
		  }CameraImgName;
		  
		  
		  class MouseCallbackData
		  {
		  public:
		    RobotConfiguration *robot_config;
		    std::string  cam_name;
		    enum ClickType
		    {
		      HEAD,
		      TAIL,
		      CENTRAL
		    } type;
		  };
		  
		  struct less_MouseCallbackData  
		  : std::binary_function< MouseCallbackData*,MouseCallbackData*,bool> 
		  {
		    bool operator()( const MouseCallbackData * a, const MouseCallbackData * b ) {
		     
		      if (  a->robot_config->name < b->robot_config->name )
			return true;
		      else if( a->robot_config->name > b->robot_config->name )
			return false;
		      else if( a->cam_name < b->cam_name )
			return true;
		      else if( a->cam_name > b->cam_name )
			return false;
		      else if(a->type < b->type)
			return true;
		      else if(a->type > b->type)
			return false;
		      
		      return false;
		    }
		  };
		  
		  // 	  
		class Guard;
		class Ball_tracker;
		class Robot_manager;
		class Camera
		{
		 
			Mutex m_mutex;
			std::string m_camera_name;
			std::string m_topic_name;
			bool m_available,m_HSV_calibration_on;
			
			ros::NodeHandle m_node;
			ros::Subscriber m_calibration_sub,m_autocalibration_sub;
			ros::Subscriber m_robot_init_pose_sub;
			ros::Publisher m_libviso_pub;
			std::vector<ros::Subscriber> m_robot_feedback_pose_sub,m_robot_feedback_GPS_sub;
			image_transport::ImageTransport m_it;
			image_transport::Subscriber m_image_sub;
			
			cv::Mat m_stream_video;
			
			//ROBOT
			double m_lost_gps_time;
			std::vector<RobotConfiguration> m_robot_array;
			
			//POINT TRASFORMATION
			float m_xCamera,m_yCamera,m_zCamera,m_R,m_omegaz,m_gammax,m_roll,m_h_robot;
			geometry_msgs::PointStamped m_camera_point;
			int m_focal_angle_x,m_focal_angle_y;
			
			// HSV
			int m_blue_threshold_on,m_green_threshold_on,m_red_threshold_on,m_yellow_threshold_on;
			int m_lb_b[3];
			int m_ub_b[3]; 
			int m_lb_g[3]; 
			int m_ub_g[3];  
			int m_lower_lb_r[3]; 
			int m_lower_ub_r[3];
			int m_upper_lb_r[3];
			int m_upper_ub_r[3]; 
			int m_lb_y[3];
			int m_ub_y[3]; 
			int m_dim_kernel_blue;
			int m_dim_kernel_green;
			int m_dim_kernel_red;
			int m_dim_kernel_yellow;
			
			// BALL DIMENSION
			int m_min_area,m_max_area;
			
			// IMAGE DIMS
			float m_image_width;
			float m_image_height;
			
			// BALLS ARRAY
			std::vector<ball_position>  m_blue_circles_W;
			std::vector<ball_position>  m_green_circles_W;
			std::vector<ball_position>  m_red_circles_W;
			std::vector<ball_position>  m_yellow_circles_W;
 
		public:
			Camera(std::string name_,std::string topic_name,
			       std::string calibration_topic,float ifovx,float ifovy);
			~Camera();
			void camera_calibration(const std_msgs::Float64MultiArray::ConstPtr& msg);
			std::vector<ball_position> cam_to_W(std::vector<ball_position>& array);
			std::vector<ball_position> charge_array(cv::Mat& img);
			void delete_thresholded_images_settings();
			void filtering(cv::Mat &src,cv::Mat &dst,int  lb[],int ub[],int dim_kernel,
			  		       int& viewing_on,std::string& color);
			void filtering(cv::Mat &src,cv::Mat &dst,int  lb[],int ub[],int dim_kernel);
			void filtering_initialization();
			void final_image_showing();
			std::vector<ball_position> get_blue_array();
			std::vector<ball_position> get_green_array();
			std::vector<ball_position> get_red_array();
			void GPS_sub(const nav_msgs::Odometry::ConstPtr& msg);
			cv::Mat get_stream_video();
			std::vector<ball_position> get_yellow_array();
			void pose_feedback(const nav_msgs::Odometry::ConstPtr& msg);
			void robot_topic_pose_subscribe(RobotConfiguration robot_pose);
			void search_ball_pos();
			void subscribe();
			void video_acquisition(const sensor_msgs::ImageConstPtr& msg);
			void thresholded_images_settings();
			ball_position W_to_cam(ball_position& in);
			void auto_recalibration(const geometry_msgs::PoseStamped& msg);
		};

	}
}


#endif // CAMERA_H