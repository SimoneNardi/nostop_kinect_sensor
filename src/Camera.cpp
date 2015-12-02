
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
#include <geometry_msgs/Pose.h>
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

 
/////////////////////////////////////////////
Camera::Camera(std::string name_,std::string image_topic_name,std::string calibration_topic)
: m_available(false)
, m_it(m_node)
, m_camera_name(name_)
, m_topic_name(image_topic_name)
, m_R(1)
, m_roll(0)
{
  ROS_INFO("CAMERA %s ON!",m_camera_name.c_str());
  m_calibration_sub = m_node.subscribe(calibration_topic,1,&Camera::camera_calibration,this);
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


void Camera::camera_calibration(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  Lock l_lock(m_mutex);
  m_roll = msg->data[0]*M_PI/180;
  m_R = msg->data[1];
  m_xCamera = msg->data[2];
  m_yCamera = msg->data[3];
  m_zCamera = msg->data[4];
  m_omegaz = msg->data[5];
  m_gammax = msg->data[6];
  m_h_robot = msg->data[7];
}
/////////////////////////////////////////////
void Camera::subscribe()
{
	cv::namedWindow(SENSOR_CV_WINDOW+m_camera_name);
	m_image_sub = m_it.subscribe(m_topic_name, 1, &Camera::video_acquisition, this, image_transport::TransportHints("raw"));	
	m_robot_init_pose_sub = m_node.subscribe<std_msgs::String>("/localizer/kinect/add_robot", 1000, &Camera::init_robot_pose,this);  
}
 
std::vector<bool> initial_pose_setted;
std::vector<cv::Rect> robot_initial_pose_rect;
std::vector<std::string> robot_name;
int robot_number=0;
bool callback_done;
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) { 
		cv::Rect pose;
		pose.height = 100;
		pose.width = 100;
		pose.x = x-pose.width/2;
		pose.y = y-pose.height/2;
		robot_initial_pose_rect.push_back(pose);
		callback_done = true;
	}
}


////////////////////////////////////////////////////////////////
void Camera::init_robot_pose(const std_msgs::String::ConstPtr& msg)
{	
    std::string name = msg->data;
    bool setted = false;
    robot_name.push_back(name);
    robot_number = robot_number+1;
    initial_pose_setted.push_back(setted);
}

////////////////////////////////////////////////////////////////
void Camera::pose_feedback(const geometry_msgs::Pose::ConstPtr& msg)
{
  std::vector<ball_position> corners_pixel,corners_cm_w;
  ball_position corn;
  float x_min,x_max,y_min,y_max;
  cv::Rect robot_position_cm,robot_position_pixel;
  corn.x = 0;
  corn.y = 0;
  corners_pixel.push_back(corn);
  corn.x = 640;
  corn.y = 0;
  corners_pixel.push_back(corn);
  corn.x = 640;
  corn.y = 480;
  corners_pixel.push_back(corn);
  corn.x = 0;
  corn.y = 480;
  corners_pixel.push_back(corn);
  corners_cm_w = cam_to_W(corners_pixel);
  robot_position_cm.x = msg->position.x;
  robot_position_cm.y = msg->position.y;
  x_min = min(corners_cm_w.at(0).x,corners_cm_w.at(1).x);
  x_max = max(corners_cm_w.at(0).x,corners_cm_w.at(1).x);
  y_min = min(corners_cm_w.at(0).y,corners_cm_w.at(1).y);
  y_max = max(corners_cm_w.at(0).y,corners_cm_w.at(1).y);
  for(size_t i=2;i<corners_cm_w.size();i++)
  {
    x_min = min(x_min,corners_cm_w.at(i).x);
    x_max = max(x_max,corners_cm_w.at(i).x);
    y_min = min(y_min,corners_cm_w.at(i).y);
    y_max = max(y_max,corners_cm_w.at(i).y);
  }
   Lock l_lock(m_mutex);
  if (robot_position_cm.x<x_max && robot_position_cm.x>x_min && robot_position_cm.y<y_max && robot_position_cm.y>y_min )
  {
    float diff;
    int to_erase;
    cv::Rect new_rect;
    robot_position_pixel = W_to_cam(robot_position_cm);
    if (robot_initial_pose_rect.size()>0)
    {
      diff = sqrt((robot_position_pixel.x-robot_initial_pose_rect.at(0).x)^2+(robot_position_pixel.y-robot_initial_pose_rect.at(0).y)^2);
      to_erase = 0;
    }
    for(size_t i = 1;i<robot_initial_pose_rect.size();i++)
    {
      if(sqrt((robot_position_pixel.x-robot_initial_pose_rect.at(i).x)^2+(robot_position_pixel.y-robot_initial_pose_rect.at(i).y)^2)<diff)
      {
	to_erase = i;
      }
    }
   
    cv::Point l_tl,l_br;
    if (robot_position_pixel.y>-240)
    {
      robot_position_pixel.height = 150;
      robot_position_pixel.width = 150;
      l_tl.x = robot_position_pixel.x-robot_position_pixel.height/2;
      l_tl.y = robot_position_pixel.y-robot_position_pixel.width/2;
      l_br.x = l_tl.x+robot_position_pixel.height;
      l_br.y = l_tl.y+robot_position_pixel.width;
      new_rect.x = l_tl.x;
      new_rect.y = l_tl.y;
      new_rect.height = robot_position_pixel.height;
      new_rect.width = robot_position_pixel.width;
      robot_initial_pose_rect.push_back(new_rect);
      robot_initial_pose_rect.erase(robot_initial_pose_rect.begin()+to_erase);
    }else{
      robot_position_pixel.height = 125;
      robot_position_pixel.width = 125;
      l_tl.x = robot_position_pixel.x-robot_position_pixel.height/2;
      l_tl.y = robot_position_pixel.y-robot_position_pixel.width/2;
      l_br.x = l_tl.x+robot_position_pixel.height;
      l_br.y = l_tl.y+robot_position_pixel.width;
      new_rect.x = l_tl.x;
      new_rect.y = l_tl.y;
      new_rect.height = robot_position_pixel.height;
      new_rect.width = robot_position_pixel.width;
      robot_initial_pose_rect.push_back(new_rect);
      robot_initial_pose_rect.erase(robot_initial_pose_rect.begin()+to_erase);
    }
  }
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
     cv::waitKey(3);
     search_ball_pos();
}



//TEST
       int k,k2;

// FILTERING INITIAL VALUES
int  lb_b[3]={100,125,100};
int  ub_b[3] = {160,255,255};
int  lb_g[3] = {30,150,50};
int  ub_g[3] = {60,255,180}; 
int  lower_lb_r[3] = {0,100,150};
int  lower_ub_r[3] = {10,255,255}; 
int  upper_lb_r[3] = {160,100,150};
int  upper_ub_r[3] = {179,255,255};
int  lb_y[3] = {20,35,135};
int  ub_y[3] = {45,255,255}; 
int dim_kernel_blue=4,dim_kernel_green=4,dim_kernel_red=4,dim_kernel_yellow=4;
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
  
     // ROBOTS INITIAL POSE
     for(int i = 0;i<robot_number;i++)
     {
      if(!initial_pose_setted[i])
      {
	std::string windows_name = robot_name[i]+" initial_pose";
	char* mouse_windows_name = new char[windows_name.size() + 1];
	std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
	mouse_windows_name[windows_name.size()] = '\0'; 
	imshow(windows_name,m_stream_video);
	cvSetMouseCallback(mouse_windows_name,mouse_callback,NULL);
	if(callback_done)
	{
	  initial_pose_setted[i] = callback_done;
	  callback_done = false;
	}	
      }else{      
	m_pose_sub = m_node.subscribe<geometry_msgs::Pose>("/"+robot_name.at(i)+"/localizer/camera/pose", 1, &Camera::pose_feedback, this);
	m_robot_feedback_pose_sub.push_back(m_pose_sub);
	std::string windows_name = robot_name[i]+" initial_pose";
	char* mouse_windows_name = new char[windows_name.size() + 1];
	std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
	mouse_windows_name[windows_name.size()] = '\0'; 
	cvDestroyWindow(mouse_windows_name);
	robot_number = robot_number-1;
	initial_pose_setted.erase(initial_pose_setted.begin()+i);
	robot_name.erase(robot_name.begin()+i);
      }
     }
     
     
     
     
     
     
     // Filtering
     // Blue Ball HSV values 
     namedWindow(BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
//      createTrackbar("H lower",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_b[0],180,0,0);
     createTrackbar("S lower",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_b[1],255,0,0);
     createTrackbar("V lower",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_b[2],255,0,0);
//      createTrackbar("H upper",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_b[0],180,0,0);
     createTrackbar("S upper",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_b[1],255,0,0);
     createTrackbar("V upper",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_b[2],255,0,0);
     createTrackbar("Kernel size",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&dim_kernel_blue,20,0,0);
     filtering(m_stream_video,m_only_blue,lb_b,ub_b,dim_kernel_blue+1);  
     
//      Green Ball HSV values
     namedWindow(GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
     createTrackbar("H lower",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_g[0],180,0,0);
     createTrackbar("S lower",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_g[1],255,0,0);
     createTrackbar("V lower",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_g[2],255,0,0);
     createTrackbar("H upper",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_g[0],180,0,0);
     createTrackbar("S upper",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_g[1],255,0,0);
     createTrackbar("V upper",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_g[2],255,0,0);
     createTrackbar("Kernel size",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&dim_kernel_green,20,0,0);
     filtering(m_stream_video,m_only_green,lb_g,ub_g,dim_kernel_green+1);  
     
//      Red Ball HSV values (H had *0.5 scale factor)
     cv::Mat l_only_lower_red, l_only_upper_red;
     namedWindow(RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
//      createTrackbar("H lower (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lower_lb_r[0],180,0,0);
     createTrackbar("S lower (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lower_lb_r[1],255,0,0);
     createTrackbar("V lower (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lower_lb_r[2],255,0,0);
//      createTrackbar("H upper (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lower_ub_r[0],180,0,0);
     createTrackbar("S upper (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lower_ub_r[1],255,0,0);
     createTrackbar("V upper (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lower_ub_r[2],255,0,0);
//      createTrackbar("H lower (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&upper_lb_r[0],180,0,0);
     createTrackbar("S lower (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&upper_lb_r[1],255,0,0);
     createTrackbar("V lower (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&upper_lb_r[2],255,0,0);
//      createTrackbar("H upper (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&upper_ub_r[0],180,0,0);
     createTrackbar("S upper (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&upper_ub_r[1],255,0,0);
     createTrackbar("V upper (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&upper_ub_r[2],255,0,0);
     createTrackbar("Kernel size",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&dim_kernel_red,20,0,0);
     filtering(m_stream_video,l_only_lower_red,lower_lb_r,lower_ub_r,dim_kernel_red+1);  
     filtering(m_stream_video,l_only_upper_red,upper_lb_r,upper_ub_r,dim_kernel_red+1);  
     cv::addWeighted(l_only_lower_red,1.0,l_only_upper_red,1.0,0.0,m_only_red);
          
//      Yellow Ball HSV values
     namedWindow(YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
//      createTrackbar("H lower",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_y[0],180,0,0);
     createTrackbar("S lower",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_y[1],255,0,0);
     createTrackbar("V lower",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&lb_y[2],255,0,0);
//      createTrackbar("H upper",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_y[0],180,0,0);
     createTrackbar("S upper",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_y[1],255,0,0);
     createTrackbar("V upper",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&ub_y[2],255,0,0);
     createTrackbar("Kernel size",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&dim_kernel_yellow,20,0,0);
     filtering(m_stream_video,m_only_yellow,lb_y,ub_y,dim_kernel_yellow+1);  
     

     Lock l_lock(m_mutex);

// //      THRESHOLDED IMG
//      imshow(BLUE_THRESHOLD_WINDOWS+m_camera_name,m_only_blue);
//      imshow(GREEN_THRESHOLD_WINDOWS+m_camera_name,m_only_green);
//      imshow(RED_THRESHOLD_WINDOWS+m_camera_name,m_only_red);
//      imshow(YELLOW_THRESHOLD_WINDOWS+m_camera_name,m_only_yellow);  
     
//      FIND BALLS ARRAY
     m_blue_circles = charge_array(m_only_blue);
     m_green_circles = charge_array(m_only_green);
     m_red_circles = charge_array(m_only_red);
     m_yellow_circles = charge_array(m_only_yellow);

       //FROM CAM (pixel) TO WORLD (cm)
     m_blue_circles_W=cam_to_W(m_blue_circles);
     m_green_circles_W=cam_to_W(m_green_circles);
     m_red_circles_W=cam_to_W(m_red_circles);
     m_yellow_circles_W=cam_to_W(m_yellow_circles);
      
     Point center;
     center.x=320;
     center.y=240;
    
     
      for(size_t i = 0;i<m_blue_circles.size();i++)
     {
       cv::Rect rec;
       ball_position a;
       a = m_blue_circles.at(i);
       rec.x = a.x;
       rec.y = a.y;
       rec.height = a.height;
       rec.width = a.width;
       rectangle(m_stream_video,rec,cv::Scalar(255, 255, 255),1,8,0);
     }
     for(size_t i = 0;i<m_green_circles.size();i++)
     {
       cv::Rect rec;
       ball_position a;
       a = m_green_circles.at(i);
       rec.x = a.x;
       rec.y = a.y;
       rec.height = a.height;
       rec.width = a.width;
       rectangle(m_stream_video,rec,cv::Scalar(255, 255, 255),1,8,0);
       
     }
     
     for(size_t i = 0;i<m_red_circles.size();i++)
     {
       cv::Rect rec;
       ball_position a;
       a = m_red_circles.at(i);
       rec.x = a.x;
       rec.y = a.y;
       rec.height = a.height;
       rec.width = a.width;
       rectangle(m_stream_video,rec,cv::Scalar(255, 255, 255),1,8,0);
    }
     
     for(size_t i = 0;i<m_yellow_circles.size();i++)
     {
       cv::Rect rec;
       ball_position a;
       a = m_yellow_circles.at(i);
       rec.x = a.x;
       rec.y = a.y;
       rec.height = a.height;
       rec.width = a.width;
       rectangle(m_stream_video,rec,cv::Scalar(255, 255, 255),1,8,0);
       
     }
     
     //// SEARCH RECTANGLE
     for(size_t i = 0;i<robot_initial_pose_rect.size();i++)
     {
       cv::Rect rec;
       rec = robot_initial_pose_rect.at(i);

       createTrackbar(" Adjoint x",SENSOR_CV_WINDOW+m_camera_name,&k,250,0,0);
       createTrackbar(" Adjoint y",SENSOR_CV_WINDOW+m_camera_name,&k2,250,0,0);
       rec.x = rec.x+k;
       rec.y = rec.y+k2;
       rectangle(m_stream_video,rec,cv::Scalar(0, 0, 0),1,8,0);
       
     }
     cv::circle(m_stream_video,center,2,cv::Scalar(0, 0, 255),-1,8,0);
     cv::circle(m_stream_video,center,10,cv::Scalar(0, 0, 0),1,8,0);
     cv::imshow(SENSOR_CV_WINDOW+m_camera_name,m_stream_video);
          
     m_blue_circles.clear();
     m_green_circles.clear();
     m_red_circles.clear();
     m_yellow_circles.clear();
}
       

void Camera::filtering(cv::Mat &src,cv::Mat &dst,int  lb[],int  ub[],int dim_kernel)
{
     // Noise smoothing
     cv::Mat blur;
     cv::GaussianBlur(src,blur, cv::Size(5, 5), 3.0, 0);

     //  HSV conversion
     cv::Mat frmHsv;
     cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
  

//       Color Thresholding
     cv::Mat rangeRes = cv::Mat::zeros(src.size(), CV_8UC1);
     cv::inRange(frmHsv, cv::Scalar(lb[0], lb[1], lb[2]),cv::Scalar(ub[0], ub[1], ub[2]),dst);

//       EROSE DILATE
      //morphological opening (remove small objects from the foreground)
      erode(dst, dst, getStructuringElement(MORPH_RECT, Size(dim_kernel, dim_kernel)) );
      dilate( dst, dst, getStructuringElement(MORPH_RECT, Size(dim_kernel, dim_kernel)) ); 

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
	cv::Point2f possible_ball;
	possible_ball.x = bBox.x;
	possible_ball.y = bBox.y;
	for(size_t j = 0;j<robot_initial_pose_rect.size();j++)
	{
         if (ratio > 0.6 && possible_ball.inside(robot_initial_pose_rect.at(j)) && bBox.area()<900 && bBox.area()>100) 
         {
	    l_ball.x = bBox.x;
	    l_ball.y = bBox.y;
	    l_ball.height = bBox.height;
	    l_ball.width = bBox.width;
	    l_array.push_back(l_ball);
	 }
	}
      }
      return l_array;
}


std::vector< ball_position > Camera::cam_to_W(std::vector<ball_position>& array)
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
   for (size_t i = 0;i<array.size();i++)
   {
      l_pos_pix.x = array[i].x;
      l_pos_pix.y = array[i].y;
      l_pos_pix.width = array[i].width;
      l_pos_pix.height = array[i].height;
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
      l_pos_cm.y= l_pos_cm.y-m_h_robot/tan(psi);
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
      Rtot[2][1] = sin(m_omegaz)*cos(m_gammax);
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
   }
   return l_out_array;
}


cv::Rect Camera::W_to_cam(cv::Rect& pos_in)
{
  cv::Rect pos_cam_pixel;
  float pos_cam_cm[3],pos_world[3],o01[3];
  float iFOV_x = (48*M_PI/180)/640;
  float iFOV_y = (32*M_PI/180)/480;
  float azimuth,elevation;
  float psi;
  float distance_from_center_x,distance_from_center_y;
  float pitch = -atan(m_R/m_zCamera)+M_PI/2;
  float R_z[3][3],R_x[3][3],Rtot[3][3];
  float x_SR_centered,y_SR_centered,x_roll_corrected,y_roll_corrected;
  pos_world[0] = pos_in.x;
  pos_world[1] = pos_in.y;
  pos_world[2] = 0;
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
  Rtot[2][1] = sin(m_omegaz)*cos(m_gammax);
  Rtot[2][2] = cos(m_gammax)*cos(m_omegaz);
  Rtot[2][3] = -sin(m_gammax);
  Rtot[3][1] = sin(m_gammax)*sin(m_omegaz);
  Rtot[3][2] = sin(m_gammax)*cos(m_omegaz);
  Rtot[3][3] = cos(m_gammax);
  pos_cam_cm[0] = Rtot[1][1]*pos_world[0]+Rtot[2][1]*pos_world[1]+Rtot[3][1]*pos_world[2]-o01[0];
  pos_cam_cm[1] = Rtot[1][2]*pos_world[0]+Rtot[2][2]*pos_world[1]+Rtot[3][2]*pos_world[2]-o01[1];
  pos_cam_cm[2] = Rtot[1][3]*pos_world[0]+Rtot[2][3]*pos_world[1]+Rtot[3][3]*pos_world[2]-o01[2];
  psi = atan(m_zCamera/pos_cam_cm[1]);
  pos_cam_cm[1]= pos_cam_cm[1]-m_h_robot/tan(psi);
  distance_from_center_y = pos_cam_cm[0];
  distance_from_center_x = -(pos_cam_cm[1]+m_R);
  azimuth = atan(distance_from_center_y/(m_R+distance_from_center_x)) ;
  elevation = atan((distance_from_center_x+m_R)/m_zCamera)-M_PI/2+pitch;
  y_roll_corrected = -elevation/iFOV_y;
  x_roll_corrected = azimuth/iFOV_x;
  x_SR_centered = x_roll_corrected*cos(m_roll)+y_roll_corrected*sin(m_roll);
  y_SR_centered = -x_roll_corrected*sin(m_roll)+y_roll_corrected*cos(m_roll);
  pos_cam_pixel.x = x_SR_centered+320.5;
  pos_cam_pixel.y = y_SR_centered+240.5;
  return pos_cam_pixel;
}

std::vector<ball_position> Camera::get_blue_array()
{
  Lock l_lock(m_mutex);
  return m_blue_circles_W;
}

std::vector<ball_position> Camera::get_green_array()
{
  Lock l_lock(m_mutex);
  return m_green_circles_W;
}

std::vector<ball_position> Camera::get_red_array()
{
  Lock l_lock(m_mutex);
  return m_red_circles_W;
}

std::vector<ball_position> Camera::get_yellow_array()
{
  Lock l_lock(m_mutex);
  return m_yellow_circles_W;
}




