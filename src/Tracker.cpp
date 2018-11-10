
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
}


cv::Rect Tracker::kalman_update(ball_position& position) 
{
  double precTick = m_ticks;
  m_ticks = (double) cv::getTickCount();
  double dT = (m_ticks-precTick) / cv::getTickFrequency(); //seconds
  cv::Rect predRect;
  
      
   m_kf.transitionMatrix.at<float>(2) = dT;
   m_kf.transitionMatrix.at<float>(9) = dT;
   cv::Point center;          
   center.x = m_state.at<float>(0);          
   center.y = m_state.at<float>(1);
      
    m_meas.at<float>(0) = position.x + position.width / 2;
    m_meas.at<float>(1) = position.y + position.height / 2;
    m_meas.at<float>(2) = (float)position.width;
    m_meas.at<float>(3) = (float)position.height;
    m_kf.correct(m_meas); // Kalman Correction
    m_state = m_kf.predict();     
    predRect.width = m_state.at<float>(4);          
    predRect.height =  m_state.at<float>(5);          
    predRect.x = m_state.at<float>(0) - predRect.width / 2;          
    predRect.y = m_state.at<float>(1) - predRect.height / 2;          
    return predRect;
}
