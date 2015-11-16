#include "Camera.h"

#include "Robot_manager.h"
#include <nostop_kinect_sensor/Camera_data.h>


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
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include "nostop_kinect_sensor/R_valueConfig.h"
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
static const float ball_radius = 7.0;
static const float h_robot = 3.5;

 
/////////////////////////////////////////////
Camera::Camera(std::string name_,std::string topic_name,std::string roll_R_topic,std::vector<float> pos_camera,float omega,float gamma)
: m_available(false)
, m_it(m_node)
, m_camera_name(name_)
, m_topic_name(topic_name)
, m_xCamera(pos_camera.at(0))
, m_yCamera(pos_camera.at(1))
, m_zCamera(pos_camera.at(2))
, m_R(1)
, m_omegaz(omega)
, m_gammax(gamma)
, m_roll(0)
{
  ROS_INFO("CAMERA %s ON!",m_camera_name.c_str());
  m_roll_R_read = m_node.subscribe(roll_R_topic,1,&Camera::roll_R_correction,this);
  subscribe();  
}

/////////////////////////////////////////////
Camera::~Camera()
{
	cv::destroyWindow(SENSOR_CV_WINDOW+m_camera_name);
	cv::destroyWindow(FILTERED_CV_WINDOW+m_camera_name);
	cv::destroyWindow(BLUE_THRESHOLD_WINDOWS+m_camera_name);
	cv::destroyWindow(GREEN_THRESHOLD_WINDOWS+m_camera_name);
	cv::destroyWindow(RED_THRESHOLD_WINDOWS+m_camera_name);
	cv::destroyWindow(YELLOW_THRESHOLD_WINDOWS+m_camera_name);
}


void Camera::roll_R_correction(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  Lock l_lock(m_mutex);
  m_roll = msg->data[0];
  m_roll = -m_roll*M_PI/180;
  m_R = msg->data[1];
}
/////////////////////////////////////////////
void Camera::subscribe()
{
	cv::namedWindow(SENSOR_CV_WINDOW+m_camera_name);
	m_image_sub = m_it.subscribe(m_topic_name, 1, &Camera::video_acquisition, this, image_transport::TransportHints("raw"));	
}


//////////////////////////////////////////
void Camera::video_acquisition(const sensor_msgs::ImageConstPtr& msg)
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

      // Update GUI Window
     
     // Update GUI Window
     cv::waitKey(3);
     // FUNCTION
     search_ball_pos();
}

/////////////////////////////////////////////////////
void Camera::search_ball_pos()
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
     

     Lock l_lock(m_mutex);
     
     m_blue_circles.clear();
     m_green_circles.clear();
     m_red_circles.clear();
     m_yellow_circles.clear();

     // THRESHOLDED IMG
     imshow(BLUE_THRESHOLD_WINDOWS+m_camera_name,m_only_blue);
//      imshow(GREEN_THRESHOLD_WINDOWS+m_camera_name,m_only_green);
     imshow(RED_THRESHOLD_WINDOWS+m_camera_name,m_only_red);
//      imshow(YELLOW_THRESHOLD_WINDOWS+m_camera_name,m_only_yellow);  
     
     // FIND BALLS ARRAY
     m_blue_circles = charge_array(m_only_blue);
//      m_green_circles = charge_array(m_only_green);
     m_red_circles = charge_array(m_only_red);
//      m_yellow_circles = charge_array(m_only_yellow);
 
     
     //FROM CAM (pixel) TO WORLD (cm)
     m_blue_circles=cam_to_W(m_blue_circles);
//      m_green_circles=cam_to_W(m_green_circles);
     m_red_circles=cam_to_W(m_red_circles);
//      m_yellow_circles=cam_to_W(m_yellow_circles);
     Point center;
     center.x=320;
     center.y=240;
     cv::circle(m_stream_video,center,2,cv::Scalar(0, 0, 255),-1,8,0);
     cv::circle(m_stream_video,center,10,cv::Scalar(0, 0, 0),1,8,0);
     cv::imshow(SENSOR_CV_WINDOW+m_camera_name,m_stream_video);

}
       


void Camera::filtering(cv::Mat &src,cv::Mat &dst,int64_t lb[],int64_t ub[])
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


std::vector<ball_position> Camera::charge_array(cv::Mat img)
{
      vector<vector<cv::Point> > l_contours; 
      ball_position l_ball;
      std::vector<ball_position> l_array;
      cv::findContours(img, l_contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);   
      for (size_t i = 0; i < l_contours.size(); i++)       
      {          
	cv::Rect bBox;   
	bBox = cv::boundingRect(l_contours[i]);            
	float ratio = (float) bBox.width / (float) bBox.height;          
	if (ratio > 1.0f)
            ratio = 1.0f / ratio;
         // Searching for a bBox almost square
         if (ratio > 0.5)// && bBox.area() >= 100)// && bBox.area() <= 10000) 
         {
	    l_ball.x = bBox.x;
	    l_ball.y = bBox.y;
	    l_ball.height = bBox.height;
	    l_ball.width = bBox.width;
         }
         l_array.push_back(l_ball);
      }
      return l_array;
}


std::vector< ball_position > Camera::cam_to_W(std::vector< ball_position >& array)
{
  std::vector<ball_position> l_out_array;
  float pos_cam[3],pos_world[3],o01[3];
  ball_position l_pos_pix,l_pos_cm,l_world_cm;
  float iFOV_x = (48*M_PI/180)/640;
  float iFOV_y = (32*M_PI/180)/480;
  float azimuth,elevation;
  float psi;
  float distance_from_center_x,distance_from_center_y;
  float pitch = -atan(m_R/m_zCamera)+M_PI/2;
   for (size_t i=0;i<array.size();i++)
   {
      l_pos_pix=array[i];
      float R_z[3][3],R_x[3][3],Rtot[3][3];
      float x_SR_centered,y_SR_centered,x_roll_corrected,y_roll_corrected;
      x_SR_centered =l_pos_pix.x-320.5;
      y_SR_centered = l_pos_pix.y-240.5;
      x_roll_corrected = x_SR_centered*cos(m_roll)-y_SR_centered*sin(m_roll);
      y_roll_corrected = x_SR_centered*sin(m_roll)+y_SR_centered*cos(m_roll);
      azimuth = x_roll_corrected*iFOV_x;
      elevation = -y_roll_corrected*iFOV_y;
      distance_from_center_x = tan(M_PI/2-pitch+elevation)*m_zCamera-m_R;
      distance_from_center_y = tan(azimuth)*(m_R+distance_from_center_x);
      l_pos_cm.x = distance_from_center_y;
      l_pos_cm.y = -(m_R+distance_from_center_x);
      l_pos_cm.height=2*ball_radius;
      l_pos_cm.width=2*ball_radius;
      psi = atan(m_zCamera/l_pos_cm.y);
      l_pos_cm.y= l_pos_cm.y-h_robot/tan(psi);
      pos_cam[0] = l_pos_cm.x;
      pos_cam[1] = l_pos_cm.y;
      pos_cam[2] = 0;
      o01[0] = m_xCamera;
      o01[1] = m_yCamera;
      o01[2] = 0;
      R_z[1][1] = cos(m_omegaz);
      R_z[1][2] = -sin(m_omegaz);
      R_z[1][3] = 0;
      R_z[2][1] = sin(m_omegaz);
      R_z[2][2] = cos(m_omegaz);
      R_z[2][3] = 0;
      R_z[3][1] = 0;
      R_z[3][2] = 0;
      R_z[3][3] = 1;
      R_x[1][1] = 1;
      R_x[1][2] = 0;
      R_x[1][3] = 0;
      R_x[2][1] = 0;
      R_x[2][2] = cos(m_gammax);
      R_x[2][3] = -sin(m_gammax);
      R_x[3][1] = 0;
      R_x[3][2] = sin(m_gammax);
      R_x[3][3] = cos(m_gammax);
      Rtot[1][1] = cos(m_omegaz);
      Rtot[1][2] = -sin(m_omegaz);
      Rtot[1][3] = 0;
      Rtot[2][1] = cos(m_omegaz)*sin(m_gammax);
      Rtot[2][2] = cos(m_gammax)*cos(m_omegaz);
      Rtot[2][3] = -sin(m_gammax);
      Rtot[3][1] = sin(m_gammax)*sin(m_omegaz);
      Rtot[3][2] = sin(m_gammax)*cos(m_omegaz);
      Rtot[3][3] = cos(m_gammax);
      pos_world[0] = Rtot[1][1]*pos_cam[0]+Rtot[1][2]*pos_cam[1]+Rtot[1][3]*pos_cam[2]+o01[0];
      pos_world[1] = Rtot[2][1]*pos_cam[0]+Rtot[2][2]*pos_cam[1]+Rtot[2][3]*pos_cam[2]+o01[1];
      pos_world[2] = Rtot[3][1]*pos_cam[0]+Rtot[3][2]*pos_cam[1]+Rtot[3][3]*pos_cam[2]+o01[2];
      l_world_cm.x = pos_world[0];
      l_world_cm.y = pos_world[1];
      l_world_cm.height = l_pos_cm.height;
      l_world_cm.width = l_pos_cm.width;
      l_out_array.push_back(l_world_cm);
//       ROS_INFO("x cam %f",pos_cam[0]);
//       ROS_INFO("y cam %f", pos_cam[1]);
//       ROS_INFO("x world %f",pos_world[0]);
//       ROS_INFO("y world %f", pos_world[1]); 
   }
   return l_out_array;
}


std::vector<ball_position> Camera::get_blue_array()
{
  Lock l_lock(m_mutex);
  return m_blue_circles;
}

std::vector<ball_position> Camera::get_green_array()
{
  Lock l_lock(m_mutex);
  return m_green_circles;
}

std::vector<ball_position> Camera::get_red_array()
{
  Lock l_lock(m_mutex);
  return m_red_circles;
}

std::vector<ball_position> Camera::get_yellow_array()
{
  Lock l_lock(m_mutex);
  return m_yellow_circles;
}