////////////////////////////////////////////////////////////
//	Collection.h
//	Created on:	07-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef COLLECTION_H
#define COLLECTION_H
#pragma once

#include <map>

#include "nostop_kinect_sensor/SensorData.h"

#include "Threads.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// KALMAN 
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "highgui.h"
#include "ros/ros.h"


namespace Robotics 
{
	namespace GameTheory
	{
		class Guard;
// 		class Tracker;
		class Robot_manager;
		struct Color
		{
		  int r;
		  int g;
		  int b;
		  
		  Color(int r_, int g_, int b_) : r(r_), g(g_), b(b_) {}
		};
		
		class ColorCompare 
		{
		  public:
		      bool operator()(const Color f,const Color s) 
		      { 
			if (f.r-s.r >0)
			  return true;
			else if (f.r-s.r <0)
			  return false;
			else if (f.g-s.g >0)
			  return true;
			else if (f.g-s.g <0)
			  return false;
			else if (f.b-s.b >0)
			  return true;
			else if (f.b-s.b <0)
			  return false;
			else
			  return false;
		      } // returns f>s
		};
		
		struct AgentSensor
		{
		  double x;
		  double y;
		  double heading;
		  
		  AgentSensor(double x_, double y_, double heading_) : x(x_), y(y_), heading(heading_) {}
		};
		 
		
		class Collection
		{
// 		  std::shared_ptr<Tracker> m_tracker_ptr_blue;
// 		  std::shared_ptr<Tracker> m_tracker_ptr_green;
// 		  std::shared_ptr<Tracker> m_tracker_ptr_red;
// 		  std::shared_ptr<Tracker> m_tracker_ptr_yellow;
		  std::shared_ptr<Robot_manager> m_robot_manager;
// 		  mutable Mutex m_mutex;
		  
		  std::map<Color, AgentSensor, ColorCompare> m_data;
		  
		  bool m_available;
		  
		  ros::NodeHandle m_node;
		  
		  image_transport::ImageTransport m_it;
		  image_transport::Subscriber m_image_sub;
		  image_transport::Subscriber m_image_sub_circles;
		  
		  ros::Subscriber m_cloud_sub;
		  
		  bool m_stream_videoFLAG;
		  cv::Mat m_stream_video;
// 		  cv::Mat m_processed;
		  cv::Mat m_circlesFounded;
		  
		  
		  //FILTERING
		  int m_min_red,m_max_red, m_min_green,m_max_green,m_min_blue,m_max_blue;
		  int m_erosion_size, m_median;
		  cv::Mat m_only_blue;
		  cv::Mat m_only_green;
		  cv::Mat m_only_red;
		  cv::Mat m_only_yellow;
		  
		  //COMPUTING ROBOT POSITION
		  float m_blue_pos[2];
		  float m_green_pos[2];
		  float m_red_pos[2];
		  float m_yellow_pos[2];
// 		  float m_robot_rb[3];
// 		  float m_robot_gy[3]; // TO DO  
		  
		public:
			Collection();
			
			~Collection();
			
// 			nostop_kinect_sensor::SensorData getMsgs();
			void subscribe();
			void searchCircles();
			void ImageFromKinect(const sensor_msgs::ImageConstPtr& msg);
			void getForeground(const sensor_msgs::ImageConstPtr& msg);
			void toPub(const sensor_msgs::ImageConstPtr& msg);
			void search_ball_pos(const sensor_msgs::ImageConstPtr& msg);
			void filtering(cv::Mat &src,cv::Mat &dst,int64_t lb[],int64_t ub[]);
// 			void robotPose(float first_ball_pos[2], float second_ball_pos[2], float robot_pose[3]);
// 			void pixel2cm(float pixel_pos[2], float cm_pos[2]);
// 			void Erosion(int erosion_elem, int erosion_size, cv::Mat const& src, cv::Mat& erosion_dst);
		};

	}
}


#endif // COLLECTOR_H