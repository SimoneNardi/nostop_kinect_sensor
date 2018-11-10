////////////////////////////////////////////////////////////
//	Collection.h
//	Created on:	07-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef COLLECTION_H
#define COLLECTION_H
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

// KALMAN 
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "ros/ros.h"
#include "some_struct.h"
#include "Robot_manager.h"


namespace Robotics 
{
	namespace GameTheory
	{
		class Guard;
		class Tracker;
		class Robot_manager;
		class Collection
		{
		 
		  
		  
		  std::string m_camera_name;
		  std::string m_topic_name;
		  bool m_available;
		  
		  ros::NodeHandle m_node;
		  
		  image_transport::ImageTransport m_it;
		  image_transport::Subscriber m_image_sub;
		  
		  cv::Mat m_stream_video;
		  
		  //FILTERING
		  cv::Mat m_only_blue;
		  cv::Mat m_only_green;
		  cv::Mat m_only_red;
		  cv::Mat m_only_yellow;
		  
		  //POINT TRASFORMATION
		  float m_xCamera,m_yCamera,m_zCamera,m_Pitch,m_omegaz,m_gammax;
		  geometry_msgs::PointStamped m_camera_point;

		  // BALLS ARRAY
		  std::vector<ball_position> m_blue_circles;
		  std::vector<ball_position> m_green_circles;
		  std::vector<ball_position> m_red_circles;
		  std::vector<ball_position>  m_yellow_circles;


		  
		public:
			Collection(std::string name_,std::string topic_name,std::vector<float> pos_camera,float pitch,float omega,float gamma);
			
			~Collection();
			
			void subscribe();
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


#endif // COLLECTOR_H