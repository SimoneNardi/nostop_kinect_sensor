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
		class Tracker;
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
		  std::shared_ptr<Tracker> m_tracker_ptr;
		  mutable Mutex m_mutex;
		  
		  std::map<Color, AgentSensor, ColorCompare> m_data;
		  
		  bool m_available;
		  
		  ros::NodeHandle m_node;
		  
		  image_transport::ImageTransport m_it;
		  image_transport::Subscriber m_image_sub;
		  image_transport::Subscriber m_image_sub_circles;
		  image_transport::Subscriber m_image_sub_photo;
		  image_transport::Publisher m_image_pub;
		  
		  ros::Subscriber m_cloud_sub;
		  
		  bool m_stream_videoFLAG;
		  cv::Mat m_stream_video;
		  cv::Mat m_processed;
		  
		  
		  int m_count;
		  int m_wait_time;//x*100 -> x*3.6 s (to do )
		  cv::Mat m_photo;
		  cv::Mat m_photo_support;
		  cv::Mat m_channel[3];
		  cv::Mat m_channel2[3];
		  cv::Mat m_channel3[3];
		  int m_red_filter,m_blue_filter,m_green_filter;
		  
		  int m_dp, m_minDist, m_param1, m_param2, m_minR, m_maxR;
		  int m_thr, m_maxval;
		  int m_min_red,m_max_red, m_min_green,m_max_green,m_min_blue,m_max_blue;
		  int m_erosion_size, m_median;

		   // KALMAN FILTER
		  double m_ticks;
		  bool m_found;
		  int m_notFoundCount = 0;
		  int m_stateSize = 6;
		  int m_measSize = 4;
		  int m_contrSize = 0;
		  unsigned int m_type = CV_32F; 
		  cv::Mat m_state;
		  cv::Mat m_meas;
		  cv::KalmanFilter m_kf;
		  
		  
		public:
			Collection();
			
			~Collection();
			
// 			nostop_kinect_sensor::SensorData getMsgs();
			void subscribe();
			void searchCircles();
			void ImageFromKinect(const sensor_msgs::ImageConstPtr& msg);
			void getForeground(const sensor_msgs::ImageConstPtr& msg);
			void toPub(const sensor_msgs::ImageConstPtr& msg);
			void search_test(const sensor_msgs::ImageConstPtr& msg);
// 			void Erosion(int erosion_elem, int erosion_size, cv::Mat const& src, cv::Mat& erosion_dst);
			void filtering(cv::Mat &src,cv::Mat &dst,int64_t lb[],int64_t ub[]);
		  
		};

	}
}


#endif // COLLECTOR_H