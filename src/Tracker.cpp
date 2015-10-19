#include <Collection.h>
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


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace cv;


Tracker::Tracker() : 
m_kf(m_stateSize,m_measSize,m_contrSize,type)
, m_state(m_stateSize,1,type)
, m_meas(m_measSize,1,type)
, m_found(false)
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


void Tracker::findCircles(cv::Mat thresholded_image, cv::Mat m_drawCircle)
{
  double precTick = m_ticks;
  m_ticks = (double) cv::getTickCount();
  double dT = (m_ticks-precTick) / cv::getTickFrequency(); //seconds
  
   if (m_found)
      {
         m_kf.transitionMatrix.at<float>(2) = dT;
         m_kf.transitionMatrix.at<float>(9) = dT;

         m_state = m_kf.predict();
	 cv::Rect predRect;          
	 predRect.width = m_state.at<float>(4);          
	 predRect.height = m_state.at<float>(5);          
	 predRect.x = m_state.at<float>(0) - predRect.width / 2;          
	 predRect.y = m_state.at<float>(1) - predRect.height / 2;            
	 cv::Point center;          
	 center.x = m_state.at<float>(0);          
	 center.y = m_state.at<float>(1);          
	 cv::circle(m_drawCircle, center, 2, CV_RGB(255,77,0), -1);            
	 cv::rectangle(m_drawCircle, predRect, CV_RGB(255,77,0), 2);       
	
      }         
     
      vector<vector<cv::Point> > l_contours;
      cv::findContours(thresholded_image, l_contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);      
      vector< vector<cv::Point> > balls;
      vector<cv::Rect> ballsBox;
      for (size_t i = 0; i < l_contours.size(); i++)       
      {          
	cv::Rect bBox;   
	bBox = cv::boundingRect(l_contours[i]);            
	float ratio = (float) bBox.width / (float) bBox.height;          
	if (ratio > 1.0f)
            ratio = 1.0f / ratio;
 
         // Searching for a bBox almost square
         if (ratio > 0.75 && bBox.area() >=650 && bBox.area() <= 1200) //To do
         {
            balls.push_back(l_contours[i]);
            ballsBox.push_back(bBox);
         }
      }
      
      cv::Point center;
      // Drawing a rectangle on a circle
      
      for (size_t i = 0; i < balls.size(); i++)
      {
         cv::drawContours(m_drawCircle, balls, i, CV_RGB(20,150,20), 1);
         cv::rectangle(m_drawCircle, ballsBox[i], CV_RGB(143,0,255), 2);
         center.x = ballsBox[i].x + ballsBox[i].width / 2;
         center.y = ballsBox[i].y + ballsBox[i].height / 2;
         cv::circle(m_drawCircle, center, 2, CV_RGB(143,0,255), -1);

      }
      // <<<<< Detection result         // >>>>> Kalman Update
      if (balls.size() == 0)
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
 
         m_meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
         m_meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
         m_meas.at<float>(2) = (float)ballsBox[0].width;
         m_meas.at<float>(3) = (float)ballsBox[0].height;
// 	    meas.at<int64>(0) = center.x;
//          meas.at<int64>(1) = center.y;
//          meas.at<int64>(2) = center.x*2;
//          meas.at<int64>(3) = center.y*2;
//  
         if (!m_found) // First detection!
         {
            // >>>> Initialization
            m_kf.errorCovPre.at<float>(0) = 1; // px
            m_kf.errorCovPre.at<float>(7) = 1; // px
            m_kf.errorCovPre.at<float>(14) = 1;
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
