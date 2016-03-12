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
#include <geometry_msgs/PointStamped.h>
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
		class RobotConfiguration
		{
		  public:
		    RobotConfiguration() {}
		    
			int pose_setted;
			cv::Point2f head_point;
			cv::Point2f tail_point;
			cv::Point2f central_point;
			cv::Point2f odom_SR_origin_pix;
			cv::Point2f odom_SR_origin_cm;
			cv::Rect pose_rect;
			bool is_magnetometer;
			std::string name;
			std::string cam_name;
			std::string cam_name_actual;
			int cam_num;
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
			bool m_available;
			
			ros::NodeHandle m_node;
			ros::Subscriber m_calibration_sub;
			ros::Subscriber m_robot_init_pose_sub;
			std::vector<ros::Subscriber> m_robot_feedback_pose_sub;
			image_transport::ImageTransport m_it;
			image_transport::Subscriber m_image_sub;
			
			cv::Mat m_stream_video;
			
			//ROBOT
			std::vector<RobotConfiguration> m_robot_array;
			std::vector<bool> m_feedback_on;
			
			//POINT TRASFORMATION
			float m_xCamera,m_yCamera,m_zCamera,m_R,m_omegaz,m_gammax,m_roll,m_h_robot;
			geometry_msgs::PointStamped m_camera_point;
			int m_focal_angle_x,m_focal_angle_y;
			
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
			std::vector<ball_position> charge_array(cv::Mat img);
			void filtering(cv::Mat &src,cv::Mat &dst,int  lb[],int ub[],int dim_kernel);
			void final_image_showing();
			std::vector<ball_position> get_blue_array();
			std::vector<ball_position> get_green_array();
			std::vector<ball_position> get_red_array();
			cv::Mat get_stream_video();
			std::vector<ball_position> get_yellow_array();
			ball_position odometry_to_srW(ball_position& robot_odometry,RobotConfiguration& robot_config);
			ball_position origin_pix2origin_world(cv::Point2f& origin_SR_pix);
			void pose_feedback(const nav_msgs::Odometry::ConstPtr& msg);
			void reset_vector();
			void robot_topic_pose_subscribe(RobotConfiguration& robot_pose);
			void search_ball_pos();
			void subscribe();
			void video_acquisition(const sensor_msgs::ImageConstPtr& msg);
			void thresholded_images_settings();
			ball_position W_to_cam(ball_position& in);
		};

	}
}


#endif // CAMERA_H