////////////////////////////////////////////////////
//	SensorCollection.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef SENSOR_COLLECTION_H
#define SENSOR_COLLECTION_H
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

// #include <pcl/io/io.h>

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
		
		class SensorCollection
		{
		  mutable Mutex m_mutex;
		  
		  std::map<Color, AgentSensor, ColorCompare> m_data;
		  
		  bool m_available;
		  
		  ros::NodeHandle m_node;
		  
		  image_transport::ImageTransport m_it;
// 		  boost::shared_ptr<image_transport::SubscriberFilter> m_image_sub;
		  image_transport::Subscriber m_image_sub;
		  
		  image_transport::Publisher m_image_pub;
		  
		  ros::Subscriber m_cloud_sub;
		  
		  bool m_foregroundFLAG;
		  cv::Mat m_foreground;
		  
		  int m_dp, m_min_dist, m_cannyEdge, m_centerDetect, m_minrad, m_maxrad;
		  int m_thr, m_maxval;
		  
		public:
			SensorCollection();
			
			~SensorCollection();
			
			nostop_kinect_sensor::SensorData getMsgs();
			
			void ImageFromKinect(const sensor_msgs::ImageConstPtr& msg);
			void getForeground(const sensor_msgs::ImageConstPtr& msg);
						
//  			void PointcloudFromKinect(const sensor_msgs::PointCloud2ConstPtr& msg);
						
			void subscribe();
			
		protected:
			//void ImageFromKinectProcess(cv::Mat const& input);
//  			void PointcloudFromKinectProcess(pcl::PointCloud< pcl::PointXYZ >::ConstPtr pcl_cloud_ );
		};

	}
}


#endif // SENSOR_COLLECTOR_H