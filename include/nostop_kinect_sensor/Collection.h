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


namespace Robotics 
{
	namespace GameTheory
	{
		class Guard;
	  
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
		  
		  bool m_foregroundFLAG;
		  cv::Mat m_foreground;
		  cv::Mat m_processed;
		  
		  int m_count;
		  int m_wait_time;//x*100 -> x*3.6 s (to do )
		  cv::Mat m_photo;
		  cv::Mat m_photo_support;
		  
		  int m_dp, m_minDist, m_param1, m_param2, m_minR, m_maxR;
		  int m_thr, m_maxval;
		  int m_min_red,m_max_red, m_min_green,m_max_green,m_min_blue,m_max_blue;		  
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
		};

	}
}


#endif // COLLECTOR_H