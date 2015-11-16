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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "ros/ros.h"
#include "some_struct.h"
#include "Robot_manager.h"
#include <nostop_kinect_sensor/Camera_data.h>
#include <nostop_kinect_sensor/R_valueConfig.h>



namespace Robotics 
{
	namespace GameTheory
	{
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
		  ros::Subscriber m_roll_R_read;
		  image_transport::ImageTransport m_it;
		  image_transport::Subscriber m_image_sub;
		  
		  cv::Mat m_stream_video;
		  
		  //FILTERING
		  cv::Mat m_only_blue;
		  cv::Mat m_only_green;
		  cv::Mat m_only_red;
		  cv::Mat m_only_yellow;
		  
		  //POINT TRASFORMATION
		  float m_xCamera,m_yCamera,m_zCamera,m_R,m_omegaz,m_gammax,m_roll;
		  geometry_msgs::PointStamped m_camera_point;

		  // BALLS ARRAY
		  std::vector<ball_position> m_blue_circles;
		  std::vector<ball_position> m_green_circles;
		  std::vector<ball_position> m_red_circles;
		  std::vector<ball_position>  m_yellow_circles;


		  
		public:
			Camera(std::string name_,std::string topic_name,std::string roll_R_topic,std::vector<float> pos_camera,float omega,float gamma);
			
			~Camera();
			
			void subscribe();
			void R_reconfigure(nostop_kinect_sensor::R_valueConfig  &config, uint32_t level);
			void roll_R_correction(const std_msgs::Float64MultiArray::ConstPtr& msg);
			void video_acquisition(const sensor_msgs::ImageConstPtr& msg);
			void search_ball_pos();
			void filtering(cv::Mat &src,cv::Mat &dst,int64_t lb[],int64_t ub[]);
			std::vector<ball_position> charge_array(cv::Mat img);
			std::vector<ball_position> cam_to_W(std::vector<ball_position> & array);
			std::vector<ball_position> get_blue_array();
			std::vector<ball_position> get_green_array();
			std::vector<ball_position> get_red_array();
			std::vector<ball_position> get_yellow_array();
		};

	}
}


#endif // CAMERA_H