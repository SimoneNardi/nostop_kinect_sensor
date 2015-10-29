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
		 
// 		struct ball_position
// 		{
// 		  double x;
// 		  double y;
// 		  double width;
// 		  double height;
// 		};
		
		class Collection
		{
		  std::shared_ptr<Robot_manager> m_robot_manager;
		  		  
		  bool m_available;
		  
		  ros::NodeHandle m_node;
		  
		  image_transport::ImageTransport m_it;
		  image_transport::Subscriber m_image_sub;
		  image_transport::Subscriber m_image_sub_circles;
		  
		  bool m_stream_videoFLAG;
		  cv::Mat m_stream_video;

		  
		  // RECTIFING
		  cv::Mat m_transmtx;
		  
		  //FILTERING
		  cv::Mat m_only_blue;
		  cv::Mat m_only_green;
		  cv::Mat m_only_red;
		  cv::Mat m_only_yellow;
		  
		  // BALLS ARRAY
		  std::vector<ball_position> m_blue_circles;
		  std::vector<ball_position> m_green_circles;
		  std::vector<ball_position> m_red_circles;
		  std::vector<ball_position>  m_yellow_circles;
		  
		  //COMPUTING ROBOT POSITION
		  float m_blue_pos[2];
		  float m_green_pos[2];
		  float m_red_pos[2];
		  float m_yellow_pos[2];
		  
		public:
			Collection();
			
			~Collection();
			
			void subscribe();
			void searchCircles();
			void getForeground(const sensor_msgs::ImageConstPtr& msg);
			void toPub(const sensor_msgs::ImageConstPtr& msg);
			void search_ball_pos(const sensor_msgs::ImageConstPtr& msg);
			void filtering(cv::Mat &src,cv::Mat &dst,int64_t lb[],int64_t ub[]);
			void balls_array(cv::Mat& blue, cv::Mat& green, cv::Mat& red, cv::Mat& yellow,
			     std::vector<ball_position>& blue_array, 
			      std::vector<ball_position>& green_array,
			      std::vector<ball_position>& red_array,
			      std::vector<ball_position>& yellow_array,cv::Mat stream);
			void charge_array(cv::Mat img, std::vector<ball_position>& array);
		};

	}
}


#endif // COLLECTOR_H