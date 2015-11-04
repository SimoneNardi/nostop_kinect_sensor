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


static const std::string SENSOR_CV_WINDOW = "Sensor view Window ";
static const std::string FILTERED_CV_WINDOW = "Filtered view Window ";
static const std::string FOUNDED_CIRCLES_WINDOW = "Founded circles Window  ";
static const std::string BLUE_THRESHOLD_WINDOWS = "Blue threshold ";
static const std::string GREEN_THRESHOLD_WINDOWS = "Green threshold ";
static const std::string RED_THRESHOLD_WINDOWS = "Red threshold ";
static const std::string YELLOW_THRESHOLD_WINDOWS = "Yellow threshold ";
static const float d_ball = 18.0;
static  cv::Point2f xy;
static  int clicked=0;
static  std::vector<cv::Point2f> corners;

 
/////////////////////////////////////////////
Collection::Collection(std::string name_)
: m_available(false)
, m_it(m_node)
, m_stream_videoFLAG(false)
, m_begin(ros::Time::now())
, m_waiting(ros::Duration(2,0))
, m_camera_name(name_)
{
  subscribe(m_camera_name);
}


/////////////////////////////////////////////
Collection::~Collection()
{
	cv::destroyWindow(SENSOR_CV_WINDOW+m_camera_name);
	cv::destroyWindow(FILTERED_CV_WINDOW+m_camera_name);
}

/////////////////////////////////////////////
void Collection::subscribe(std::string camera_name)
{	
	std::string l_camera_name;
	cv::namedWindow(SENSOR_CV_WINDOW+m_camera_name);
	l_camera_name = m_camera_name.substr(0,m_camera_name.find("_"));
	if(l_camera_name == "kinect")
	{	ROS_INFO("METTO KINECT");
	  m_image_sub = m_it.subscribe("/camera/rgb/image_rect_color", 1, &Collection::video_acquisition, this, image_transport::TransportHints("raw"));
	} else { ROS_INFO("USB CAM");
	  m_image_sub = m_it.subscribe("/usb_cam/image_raw", 1, &Collection::video_acquisition, this, image_transport::TransportHints("raw"));
	}
	m_stream_videoFLAG = false;
  
}


//////////////////////////////////////////
void Collection::video_acquisition(const sensor_msgs::ImageConstPtr& msg)
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
     //RECTIFY
     if(!m_transmtx.empty())// && corners.size() == 4)
    {
      cv::warpPerspective(m_stream_video, m_rectified_img, m_transmtx, m_rectified_img.size());
      m_rectified_img.copyTo(m_stream_video);
    }else{
          img_rectify(); 
    }
  
  m_stream_videoFLAG = true;
    
   if(m_stream_videoFLAG)
   {
      // Update GUI Window
      cv::imshow(SENSOR_CV_WINDOW+m_camera_name,m_stream_video);
      cv::waitKey(3);     
}

// FUNCTION
search_ball_pos();
}

//////////////////////////////////////////////////////
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	//This is called every time a mouse event occurs in the window
	if (event == CV_EVENT_LBUTTONDBLCLK) { //This is executed when the left mouse button is clicked
		//Co-ordinates of the left click are assigned to global variables and flag is set to 1
		xy.x = x;
		xy.y = y;
		corners.push_back(xy);
		clicked = clicked+1;
	}
}
void Collection::img_rectify()
{
  
  if(ros::Time::now()-m_begin < m_waiting)
     {
       m_stream_video.copyTo(m_rectified_img);
     }else{
            if(m_transmtx.empty())
	      {
		cvNamedWindow("Frame",CV_WINDOW_AUTOSIZE); //Window is created for image of each frame
		imshow("Frame",m_rectified_img);
		cvSetMouseCallback("Frame",mouse_callback,NULL);
		if(corners.size()==4)
		{
		  // Corners of the destination image
		  std::vector<cv::Point2f> quad_pts;
		  quad_pts.push_back(cv::Point2f(0, 0));
		  quad_pts.push_back(cv::Point2f(m_rectified_img.cols, 0));
		  quad_pts.push_back(cv::Point2f(m_rectified_img.cols, m_rectified_img.rows));
		  quad_pts.push_back(cv::Point2f(0, m_rectified_img.rows));
		  // Get transformation matrix
		  m_transmtx = cv::getPerspectiveTransform(corners, quad_pts);
		  corners.clear();
		  }
	    }
	  }
}

/////////////////////////////////////////////////////
void Collection::search_ball_pos()
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
     lb_b[1] = 125;
     lb_b[2] = 100;
     ub_b[0] = 320*0.5;
     ub_b[1] = 255;
     ub_b[2] = 255;
     filtering(m_stream_video,m_only_blue,lb_b,ub_b);  
     
     // Green Ball HSV values (H had *0.5 scale factor)
     int64_t lb_g[3],ub_g[3]; 
     lb_g[0] = 30; 
     lb_g[1] = 150;
     lb_g[2] = 50;
     ub_g[0] = 60;
     ub_g[1] = 255;
     ub_g[2] = 180;
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
     lb_y[0] = 20; 
     lb_y[1] = 115;
     lb_y[2] = 135;
     ub_y[0] = 45;
     ub_y[1] = 255;
     ub_y[2] = 255;
     filtering(m_stream_video,m_only_yellow,lb_y,ub_y);  
          
     m_blue_circles.clear();
     m_green_circles.clear();
     m_red_circles.clear();
     m_yellow_circles.clear();
    
     // THRESHOLDED IMG
     imshow(BLUE_THRESHOLD_WINDOWS+m_camera_name,m_only_blue);
     imshow(GREEN_THRESHOLD_WINDOWS+m_camera_name,m_only_green);
     imshow(RED_THRESHOLD_WINDOWS+m_camera_name,m_only_red);
     imshow(YELLOW_THRESHOLD_WINDOWS+m_camera_name,m_only_yellow);  
     
     // FIND BALLS ARRAY
     balls_array(
       m_only_blue,m_only_green,m_only_red,m_only_yellow,
       m_blue_circles,m_green_circles,m_red_circles,m_yellow_circles,
       m_stream_video);
     
     //FROM PIXEL TO CM
      m_blue_circles=pixel_to_cm(m_blue_circles);
     m_green_circles=pixel_to_cm(m_green_circles);
     m_red_circles=pixel_to_cm(m_red_circles);
     m_yellow_circles=pixel_to_cm(m_yellow_circles);

     //FROM CAMERA TO WORLD
     m_blue_circles=pos_transformation(m_blue_circles);
     m_green_circles=pos_transformation(m_green_circles);
     m_red_circles=pos_transformation(m_red_circles);
     m_yellow_circles=pos_transformation(m_yellow_circles);
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
     
//     >>>>> Improving the result
     cv::erode(rangeRes, dst, cv::Mat(), cv::Point(-1, -1), 2);
     cv::dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), 2);    
     
}



void Collection::balls_array(cv::Mat& blue, cv::Mat& green, cv::Mat& red, cv::Mat& yellow,
			     std::vector<ball_position>& blue_array, 
			      std::vector<ball_position>& green_array,
			      std::vector<ball_position>& red_array,
			      std::vector<ball_position>& yellow_array,cv::Mat stream)
{ 
  // charge_array(blue,blue_array);
//    charge_array(green,green_array);
   charge_array(red,red_array);
//    charge_array(yellow,yellow_array);

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
         if (ratio > 0.3) //&& bBox.area() >= 500)// && bBox.area() <= 10000) 
         {
	    l_ball.x = bBox.x;
	    l_ball.y = bBox.y;
	    l_ball.height = bBox.height;
	    l_ball.width = bBox.width;
         }
         array.push_back(l_ball);
      }
}


vector< ball_position > Collection::pixel_to_cm(vector< ball_position >& array)
{
  std::vector<ball_position> l_out_array;
  ball_position l_pos_pix,l_pos_cm;
  float pixel;
   for (size_t i=0;i<array.size();i++)
   {
      l_pos_pix=array[i];
      pixel=d_ball/((l_pos_pix.width+l_pos_pix.height)/2);
      l_pos_cm.x=pixel*l_pos_pix.x;
      l_pos_cm.y=pixel*l_pos_pix.y;
      l_pos_cm.height=pixel*l_pos_pix.height;
      l_pos_cm.width=pixel*l_pos_pix.width;
      l_out_array.push_back(l_pos_cm);
       ROS_INFO("%f",l_pos_pix.x);
       ROS_INFO("%f", l_pos_pix.y);
       ROS_INFO("%f",l_pos_cm.x);
       ROS_INFO("%f", l_pos_cm.y);
       ROS_INFO("%f",l_pos_pix.height);
       ROS_INFO("%f", l_pos_pix.width);
       ROS_INFO("%f",l_pos_cm.height);
       ROS_INFO("%f", l_pos_cm.width);
       
   }
   return l_out_array;
}


vector< ball_position > Collection::pos_transformation(vector< ball_position >& array)  //TO COMPLETE
{
    std::vector<ball_position> l_out_array;
    ball_position l_pos;
    geometry_msgs::PointStamped l_world_point;
    l_world_point.header.frame_id="world_frame";
    m_broadcaster.sendTransform(tf::StampedTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),
											tf::Vector3(10,10,0)),ros::Time::now(),"world_frame","camera")));
    m_camera_point.header.frame_id = "camera";

   for (size_t i=0;i<array.size();i++)
    {
      m_camera_point.header.stamp = ros::Time(); //To use the most recent trasformation
      m_camera_point.point.x= array[i].x;
      m_camera_point.point.y= array[i].y;
      m_camera_point.point.z= 0;
      
      
  //    m_listener.transformPoint("world_frame", m_camera_point, l_world_point);
//       l_pos.x=l_world_point.point.x;
//       l_pos.y=l_world_point.point.y;
//       l_pos.height=array[i].height;
//       l_pos.width=array[i].width;
//       l_out_array.push_back(l_pos);
//       if (i==0)
//       {
//       ROS_INFO("camera ---> %f", array[i].x);
//       ROS_INFO("world ---> %f", l_pos.x);
//       }
    }
  return l_out_array;
}

std::vector<ball_position> Collection::get_blue_array()
{
  return m_blue_circles;
}

std::vector<ball_position> Collection::get_green_array()
{
  return m_green_circles;
}

std::vector<ball_position> Collection::get_red_array()
{
  return m_red_circles;
}

std::vector<ball_position> Collection::get_yellow_array()
{
  return m_yellow_circles;
}