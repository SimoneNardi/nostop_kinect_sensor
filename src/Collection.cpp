#include "Collection.h"

#include "Robot_manager.h"


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
#include <math.h>
#include <iostream>
 
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace cv;


static const std::string SENSOR_CV_WINDOW = "Sensor view Window";
static const std::string FILTERED_CV_WINDOW = "Filtered view Window";
static const std::string FOUNDED_CIRCLES_WINDOW = "Founded circles Window";

 
/////////////////////////////////////////////
Collection::Collection()
: m_available(false)
, m_it(m_node)
, m_stream_videoFLAG(false)
, m_robot_manager(nullptr)
{
	m_robot_manager = std::make_shared<Robot_manager>();
	m_robot_manager->subscribe();
}


/////////////////////////////////////////////
Collection::~Collection()
{
	cv::destroyWindow(SENSOR_CV_WINDOW);
	cv::destroyWindow(FILTERED_CV_WINDOW);
}

/////////////////////////////////////////////
void Collection::subscribe()
{	
	
	ROS_INFO("Sensor: Collection subscribe!");
	cv::namedWindow(SENSOR_CV_WINDOW);
	m_image_sub = m_it.subscribe("/camera/rgb/image_rect_color", 1, &Collection::getForeground, this, image_transport::TransportHints("raw"));
	
	m_stream_videoFLAG = false;
	std::cout << "Sensor: ForeGround Collected!"<< std::endl << std::flush;
  
}

//////////////////////////////////////////
void Collection::getForeground(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
	cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    else
	cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  
  m_stream_video = cv_ptr->image.clone();
  
  m_stream_videoFLAG = true;
    
   if(m_stream_videoFLAG)
   {
      // Update GUI Window
      cv::imshow(SENSOR_CV_WINDOW,m_stream_video);
      cv::waitKey(3);     
}
}


/////////////////////////////////////////////////////
void Collection::searchCircles()
{	
	ROS_INFO("Searching init.");
	m_image_sub_circles = m_it.subscribe("/camera/rgb/image_rect_color", 1, &Collection::search_ball_pos, this, image_transport::TransportHints("raw"));
}

void Collection::search_ball_pos(const sensor_msgs::ImageConstPtr& msg)
{ 
       cv::Mat withCircle_blue,withCircle_green,withCircle_red,withCircle_yellow,l_bg,l_ry;
//      withCircle_blue=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type);
     withCircle_green=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     withCircle_red=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     withCircle_yellow=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     m_only_blue=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     m_only_green=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     m_only_red=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     m_only_yellow=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     l_bg=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     l_ry==Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
     m_stream_video.copyTo(withCircle_blue);

     
     // Filtering
     // Blue Ball HSV values (H had *0.5 scale factor)
     int64_t lb_b[3],ub_b[3]; 
     lb_b[0] = 200*0.5; 
     lb_b[1] = 140;
     lb_b[2] = 140;
     ub_b[0] = 320*0.5;
     ub_b[1] = 255;
     ub_b[2] = 255;
     filtering(m_stream_video,m_only_blue,lb_b,ub_b);  
     
     // Green Ball HSV values (H had *0.5 scale factor)
     int64_t lb_g[3],ub_g[3]; 
     lb_g[0] = 30; 
     lb_g[1] = 100;
     lb_g[2] = 50;
     ub_g[0] = 60;
     ub_g[1] = 255;
     ub_g[2] = 255;
     filtering(m_stream_video,m_only_green,lb_g,ub_g);  
     
     // Red Ball HSV values (H had *0.5 scale factor)
     cv::Mat l_only_lower_red, l_only_upper_red;
     int64_t lb_r[3],ub_r[3]; 
     lb_r[0] = 0; 
     lb_r[1] = 100;
     lb_r[2] = 150;
     ub_r[0] = 10;
     ub_r[1] = 255;
     ub_r[2] = 255;
     filtering(m_stream_video,l_only_lower_red,lb_r,ub_r);  
     
     lb_r[0] = 160; 
     lb_r[1] = 100;
     lb_r[2] = 150;
     ub_r[0] = 179;
     ub_r[1] = 255;
     ub_r[2] = 255;
     filtering(m_stream_video,l_only_upper_red,lb_r,ub_r);  
     cv::addWeighted(l_only_lower_red,1.0,l_only_upper_red,1.0,0.0,m_only_red);
          
     // Yellow Ball HSV values (H had *0.5 scale factor)
     int64_t lb_y[3],ub_y[3]; 
     lb_y[0] = 15; 
     lb_y[1] = 70;
     lb_y[2] = 100;
     ub_y[0] = 45;
     ub_y[1] = 255;
     ub_y[2] = 255;
     filtering(m_stream_video,m_only_yellow,lb_y,ub_y);  
          
     m_blue_circles.clear();
     m_green_circles.clear();
     m_red_circles.clear();
     m_yellow_circles.clear();
     
     // FIND BALLS ARRAY
     balls_array(
       m_only_blue,m_only_green,m_only_red,m_only_yellow,
       m_blue_circles,m_green_circles,m_red_circles,m_yellow_circles,
       m_stream_video);
    
     
     
     
     std::vector<cv::Point2f> quad_pts,corners;
     	

	//TODO corners!!!!!!!
	quad_pts.push_back(cv::Point2f(0, 0));
	quad_pts.push_back(cv::Point2f(m_stream_video.cols, 0));
	quad_pts.push_back(cv::Point2f(m_stream_video.cols, m_stream_video.rows));
	quad_pts.push_back(cv::Point2f(0, m_stream_video.rows));
	corners = quad_pts;
	cv::Mat dst=Mat::zeros(m_stream_video.rows,m_stream_video.cols,CV_32F);
	m_transmtx = cv::getPerspectiveTransform(corners, quad_pts);
	
	cv::warpPerspective(m_stream_video, dst, m_transmtx,dst.size());
	imshow("TEST",dst);
	
     
}


void Collection::filtering(cv::Mat &src,cv::Mat &dst,int64_t lb[],int64_t ub[])
{
     // Noise smoothing
     cv::Mat blur;
     cv::GaussianBlur(src, blur, cv::Size(5, 5), 3.0, 3.0);
     
     //  HSV conversion
     cv::Mat frmHsv;
     cv::cvtColor(blur, frmHsv, CV_BGR2HSV);

     //  Color Thresholding
     cv::Mat rangeRes = cv::Mat::zeros(src.size(), CV_8UC1);
     cv::inRange(frmHsv, cv::Scalar(lb[0], lb[1], lb[2]),cv::Scalar(ub[0], ub[1], ub[2]), rangeRes);

     // >>>>> Improving the result
     cv::erode(rangeRes, dst, cv::Mat(), cv::Point(-1, -1), 2);
     cv::dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), 2);    
     
}



void Collection::balls_array(cv::Mat& blue, cv::Mat& green, cv::Mat& red, cv::Mat& yellow,
			     std::vector<ball_position>& blue_array, 
			      std::vector<ball_position>& green_array,
			      std::vector<ball_position>& red_array,
			      std::vector<ball_position>& yellow_array,cv::Mat stream)
{ 
   charge_array(blue,blue_array);
   charge_array(green,green_array);
   charge_array(red,red_array);
   charge_array(yellow,yellow_array);
   m_robot_manager->array_assignment(blue_array,green_array,red_array,yellow_array,m_stream_video);
}

void Collection::charge_array(cv::Mat img, std::vector<ball_position>& array)
{
      vector<vector<cv::Point> > l_contours; 
      ball_position l_ball;
      cv::findContours(img, l_contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);   
      for (size_t i = 0; i < l_contours.size(); i++)       
      {          
	cv::Rect bBox;   
	bBox = cv::boundingRect(l_contours[i]);            
	float ratio = (float) bBox.width / (float) bBox.height;          
	if (ratio > 1.0f)
            ratio = 1.0f / ratio;
         // Searching for a bBox almost square
         if (ratio > 0.85 && bBox.area() >=200 && bBox.area() <= 1000) 
         {
	    l_ball.x = bBox.x;
	    l_ball.y = bBox.y;
	    l_ball.height = bBox.height;
	    l_ball.width = bBox.width;
         }
         array.push_back(l_ball);
      }
}


