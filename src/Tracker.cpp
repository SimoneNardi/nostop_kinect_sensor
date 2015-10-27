
#include "Collection.h"
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
#include <iostream>
#include "Tracker.h"
#include <std_msgs/Float32.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace cv;


Tracker::Tracker() : 
m_kf(m_stateSize,m_measSize,m_contrSize,type)
, m_state(m_stateSize,1,type)
, m_meas(m_measSize,1,type)
, m_found(false)
, m_close(true)
{
  matrixSettings(m_kf);
}

Tracker::~Tracker() {}

void Tracker::matrixSettings(cv::KalmanFilter m_kf)
{ 
  cv::setIdentity(m_kf.transitionMatrix);
  m_kf.measurementMatrix = cv::Mat::zeros(m_measSize, m_stateSize, type);
  m_kf.measurementMatrix.at<float>(0) = 1.0f;
  m_kf.measurementMatrix.at<float>(7) = 1.0f;
  m_kf.measurementMatrix.at<float>(16) = 1.0f;
  m_kf.measurementMatrix.at<float>(23) = 1.0f;
  
   cv::setIdentity(m_kf.processNoiseCov, cv::Scalar(1e-2));
  m_kf.processNoiseCov.at<float>(0) = 1e-2;
  m_kf.processNoiseCov.at<float>(7) = 1e-2;
  m_kf.processNoiseCov.at<float>(14) = 2.0f;
  m_kf.processNoiseCov.at<float>(21) = 1.0f;
  m_kf.processNoiseCov.at<float>(28) = 1e-2;
  m_kf.processNoiseCov.at<float>(35) = 1e-2;
 
   // Measures Noise Covariance Matrix R
   cv::setIdentity(m_kf.measurementNoiseCov, cv::Scalar(1e-1));
}


void Tracker::kalman_update(ball_position position)
{
  double precTick = m_ticks;
  m_ticks = (double) cv::getTickCount();
  double dT = (m_ticks-precTick) / cv::getTickFrequency(); //seconds
  
  cv::Rect predRect;
   
  if (m_found) //TO DO 
      {
         m_kf.transitionMatrix.at<float>(2) = dT;
         m_kf.transitionMatrix.at<float>(9) = dT;
         m_state = m_kf.predict();
	          
	 predRect.width = m_state.at<float>(4);          
	 predRect.height = m_state.at<float>(5);          
	 predRect.x = m_state.at<float>(0) - predRect.width / 2;          
	 predRect.y = m_state.at<float>(1) - predRect.height / 2;            
	 cv::Point center;          
	 center.x = m_state.at<float>(0);          
	 center.y = m_state.at<float>(1);          
      }
      
      if (m_found== 0)// TO DO 
      {
         m_notFoundCount++;
	 if( m_notFoundCount >= 10 )
         {
            m_found = false;
         }
         else
            m_kf.statePost = m_state;
      }
      else
      {
         m_notFoundCount = 0;
 
         m_meas.at<float>(0) = position.x + position.width / 2;
         m_meas.at<float>(1) = position.y + position.height / 2;
         m_meas.at<float>(2) = (float)position.width;
         m_meas.at<float>(3) = (float)position.height;

         if (!m_found) // First detection!
         {
            // >>>> Initialization
            m_kf.errorCovPre.at<float>(0) = 1; // px
            m_kf.errorCovPre.at<float>(7) = 1; // px
            m_kf.errorCovPre.at<float>(14) = 1;m_kf.statePost = m_state;
            m_kf.errorCovPre.at<float>(21) = 1;
            m_kf.errorCovPre.at<float>(28) = 1; // px
            m_kf.errorCovPre.at<float>(35) = 1; // px
 
            m_state.at<float>(0) = m_meas.at<float>(0);
            m_state.at<float>(1) = m_meas.at<float>(1);
            m_state.at<float>(2) = 0;
            m_state.at<float>(3) = 0;
            m_state.at<float>(4) = m_meas.at<float>(2);
            m_state.at<float>(5) = m_meas.at<float>(3);
            // <<<< Initialization
 
            m_found = true;

	    
         }
         else
            m_kf.correct(m_meas); // Kalman Correction
      }
 }
 

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 /*
void Tracker::write2bag(std::string color)
{
    if(m_close){
      m_bag.open(color+".bag",rosbag::bagmode::Write);
      m_close = false;
    }
    if (m_found)
    {
      std_msgs::Float32 x_bag,y_bag;
      float x,y;
      x = m_state.at<float>(0);
      y = m_state.at<float>(1);
      x_bag.data = x;
      y_bag.data = y;
      m_bag.write<std_msgs::Float32>("x",ros::Time::now(),x_bag);
      m_bag.write<std_msgs::Float32>("y",ros::Time::now(),y_bag);
      } else{
	if(m_notFoundCount>=10){
	    m_bag.close();
	    m_close = true;
	  }else{
	     std_msgs::Float32 x_bag,y_bag;
	    float x,y;
	    x = m_state.at<float>(0);
	    y = m_state.at<float>(1);
	    x_bag.data = x;
	    y_bag.data = y;
	    m_bag.write<std_msgs::Float32>("x",ros::Time::now(),x_bag);
	    m_bag.write<std_msgs::Float32>("y",ros::Time::now(),y_bag);
	    }
      } 
}
      
void Tracker::bag2read(std::string color)
{ 
  m_bag.open(color+".bag",rosbag::bagmode::Read);
  float x;
  std::vector<std::string> topics;
  topics.push_back(std::string("x"));
  topics.push_back(std::string("y"));
  
  rosbag::View view(m_bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m,view)
  {
    std_msgs::Float32::ConstPtr s = m.instantiate<std_msgs::Float32>();
    x = s->data;
    
    ROS_INFO("%f",x);
  }
  m_bag.close();
}
*/
