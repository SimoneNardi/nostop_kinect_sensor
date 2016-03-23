
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
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <iostream>


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace cv;

// WINDOWS
static const std::string SENSOR_CV_WINDOW = "Sensor view Window ";
static const std::string FILTERED_CV_WINDOW = "Filtered view Window ";
static const std::string FOUNDED_CIRCLES_WINDOW = "Founded circles Window  ";
static const std::string BLUE_THRESHOLD_WINDOWS = "Blue threshold ";
static const std::string GREEN_THRESHOLD_WINDOWS = "Green threshold ";
static const std::string RED_THRESHOLD_WINDOWS = "Red threshold ";
static const std::string YELLOW_THRESHOLD_WINDOWS = "Yellow threshold ";

static const float g_ball_radius = 3.5;



/// CONSTRUCTOR
Camera::Camera(std::string name_,std::string image_topic_name,std::string calibration_topic,float ifovx,float ifovy)
: m_available(false)
, m_it(m_node)
, m_camera_name(name_)
, m_topic_name(image_topic_name)
, m_R(1)
, m_roll(0)
, m_focal_angle_x(ifovx)
, m_focal_angle_y(ifovy)
{
	filtering_initialization();
	ROS_INFO("CAMERA %s ON!",m_camera_name.c_str());
	m_calibration_sub = m_node.subscribe(calibration_topic,10,&Camera::camera_calibration,this);
	subscribe();  
}

void Camera::filtering_initialization()
{
	Lock l_lock(m_mutex);
	// FILTERING INITIAL VALUES (H,S,V)
	// BLUE
	m_lb_b[0] = 100;
	m_lb_b[1] = 125;
	m_lb_b[2] = 100;
	m_ub_b[0] = 160;
	m_ub_b[1] = 255;
	m_ub_b[2] = 255;
	//GREEN
	m_lb_g[0] = 30;
	m_lb_g[1] = 150;
	m_lb_g[2] = 50;
	m_ub_g[0] = 60;
	m_ub_g[1] = 255;
	m_ub_g[2] = 180;
	//RED
	m_lower_lb_r[0] = 0;
	m_lower_lb_r[1] = 170;
	m_lower_lb_r[2] = 150;
	m_lower_ub_r[0] = 10;
	m_lower_ub_r[1] = 255;
	m_lower_ub_r[2] = 255;
	m_upper_lb_r[0] = 160;
	m_upper_lb_r[1] = 100;
	m_upper_lb_r[2] = 150;
	m_upper_ub_r[0] = 179;
	m_upper_ub_r[1] = 255;
	m_upper_ub_r[2] = 255;
	//YELLOW
	m_lb_y[0] = 20;
	m_lb_y[1] = 50;
	m_lb_y[2] = 160;
	m_ub_y[0] = 45;
	m_ub_y[1] = 255;
	m_ub_y[2] = 255;
	//KERNEL
	m_dim_kernel_blue = 4;
	m_dim_kernel_green = 4;
	m_dim_kernel_red = 4;
	m_dim_kernel_yellow = 4;
}

/// DISTRUCTOR
Camera::~Camera()
{
	cv::destroyWindow(SENSOR_CV_WINDOW+m_camera_name);
	cv::destroyWindow(FILTERED_CV_WINDOW+m_camera_name);
	cv::destroyWindow(BLUE_THRESHOLD_WINDOWS+m_camera_name);
	cv::destroyWindow(GREEN_THRESHOLD_WINDOWS+m_camera_name);
	cv::destroyWindow(RED_THRESHOLD_WINDOWS+m_camera_name);
	cv::destroyWindow(YELLOW_THRESHOLD_WINDOWS+m_camera_name);
}



// FROM PIXEL BALL POSITION TO WORLD BALL POSITION
std::vector< ball_position > Camera::cam_to_W(std::vector<ball_position>& array)
{
	std::vector<ball_position> l_out_array;
	float pos_cam[3],pos_world[3],o01[3];
	float Rtot[3][3];
	float x_SR_centered,y_SR_centered,x_roll_corrected,y_roll_corrected;
	ball_position l_pos_pix,l_pos_cm,l_world_cm;
	float iFOVx = (m_focal_angle_x*M_PI/180)/640;
	float iFOVy = (m_focal_angle_y*M_PI/180)/480;
	float azimuth,elevation;
	float phi;
	float distance_from_center_x,distance_from_center_y;
	float pitch = -atan(m_R/m_zCamera)+M_PI/2;
	float psi1,psi2,rho,ipotenuse,rx;
	for (size_t i = 0;i<array.size();i++)
	{
		l_pos_pix.x = array[i].x;
		l_pos_pix.y = array[i].y;
		l_pos_pix.width = array[i].width;
		l_pos_pix.height = array[i].height;    
		x_SR_centered =l_pos_pix.x-320.5;
		y_SR_centered = l_pos_pix.y-240.5;
		x_roll_corrected = x_SR_centered*cos(m_roll)-y_SR_centered*sin(m_roll);
		y_roll_corrected = x_SR_centered*sin(m_roll)+y_SR_centered*cos(m_roll);

		// ANGLES
		azimuth = x_roll_corrected*iFOVx;
		elevation = abs(y_roll_corrected)*iFOVy;      
		ipotenuse = sqrt(pow(m_zCamera,2)+pow(m_R,2));
		// // SR IN CENTER OF VIEW

		if(y_roll_corrected>0)
		{
			// y correction
			rho = M_PI-elevation-pitch; 
			distance_from_center_y = (sin(elevation)/sin(rho))*ipotenuse;
			// h robot correction
			psi2 = atan((m_R-distance_from_center_y)/m_zCamera);
			distance_from_center_y = distance_from_center_y+m_h_robot*tan(psi2);
			// x correction
			rx = (sin(pitch)/sin(rho))*ipotenuse;
			// h robot correction in x
			rx = rx-sqrt(pow(m_h_robot,2)+pow(m_h_robot*tan(psi2),2));
			distance_from_center_x = rx*tan(azimuth);
		}else{
			// y correction
			phi = pitch-elevation;
			distance_from_center_y = -(sin(elevation)/sin(phi))*ipotenuse;
			// h robot correction in y
			psi1 = atan((m_R-distance_from_center_y)/m_zCamera);
			distance_from_center_y = distance_from_center_y+m_h_robot*tan(psi1);
			// x correction
			rx = (sin(M_PI-pitch)/sin(phi))*ipotenuse;
			// h robot correction in x
			rx = rx-sqrt(pow(m_h_robot,2)+pow(m_h_robot*tan(psi1),2));
			distance_from_center_x = rx*tan(azimuth);
		} 

		// SR UNDER CAMERA 
		l_pos_cm.x = distance_from_center_x;
		l_pos_cm.y = -(m_R-distance_from_center_y);
		l_pos_cm.height=2*g_ball_radius;
		l_pos_cm.width=2*g_ball_radius;
		
		pos_cam[0] = l_pos_cm.x;
		pos_cam[1] = l_pos_cm.y;
		pos_cam[2] = 0;
		o01[0] = m_xCamera;
		o01[1] = m_yCamera;
		o01[2] = 0;
	  //         Rz * Rx ( current axis ) 
		Rtot[1][1] = cos(m_omegaz);
		Rtot[1][2] = -cos(m_gammax)*sin(m_omegaz);
		Rtot[1][3] = sin(m_gammax)*sin(m_omegaz);
		Rtot[2][1] = sin(m_omegaz);
		Rtot[2][2] = cos(m_gammax)*cos(m_omegaz);
		Rtot[2][3] = -sin(m_gammax)*cos(m_omegaz);
		Rtot[3][1] = 0;
		Rtot[3][2] = sin(m_gammax);
		Rtot[3][3] = cos(m_gammax);
		
		// ROTATION
		pos_world[0] = Rtot[1][1]*pos_cam[0]+Rtot[1][2]*pos_cam[1]+Rtot[1][3]*pos_cam[2];
		pos_world[1] = Rtot[2][1]*pos_cam[0]+Rtot[2][2]*pos_cam[1]+Rtot[2][3]*pos_cam[2];
		pos_world[2] = Rtot[3][1]*pos_cam[0]+Rtot[3][2]*pos_cam[1]+Rtot[3][3]*pos_cam[2];

		// TRASLATION
		l_world_cm.x = pos_world[0]+o01[0];
		l_world_cm.y = pos_world[1]+o01[1];
		l_world_cm.height = l_pos_cm.height;
		l_world_cm.width = l_pos_cm.width;
		l_out_array.push_back(l_world_cm);
	}
	
	return l_out_array;
}



/// CAMERA HARDWARE CALIBRATION 
void Camera::camera_calibration(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	Lock l_lock(m_mutex);
	ROS_INFO("%s published calibration values",m_camera_name.c_str());
	m_roll = msg->data[0]*M_PI/180;
	m_R = msg->data[1];
	m_xCamera = msg->data[2];
	m_yCamera = msg->data[3];
	m_zCamera = msg->data[4];
	m_omegaz = msg->data[5];// radians
	ROS_INFO("omegaz--->%f",m_omegaz*180/M_PI);
	m_gammax = msg->data[6]*M_PI/180;
	m_h_robot = msg->data[7];
}
 
/// CHARGE CLASS ARRAY WITH FOUNDED BALL POSITION
std::vector<ball_position> Camera::charge_array(cv::Mat img)
{
	std::vector<vector<cv::Point> > l_contours; 
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
		possible_ball.x = bBox.x+bBox.width/2;
		possible_ball.y = bBox.y+bBox.height/2;

		for(size_t j = 0;j<m_robot_array.size();j++)
		{
			if (ratio > 0.6 && 
			    m_robot_array[j].pose_setted == 3 && 
			    possible_ball.inside(m_robot_array[j].pose_rect) && 
			    bBox.area()<1900 && 
			    bBox.area()>100) 
			{
				cv::Point point;
				point.x = possible_ball.x;
				point.y = possible_ball.y;
				circle(m_stream_video,point,2,cv::Scalar(0, 0, 0),-1,8,0);
				rectangle(m_stream_video,bBox,cv::Scalar(255, 255, 255),1,8,0);
				l_ball.x = possible_ball.x;
				l_ball.y = possible_ball.y;
				l_ball.height = bBox.height;
				l_ball.width = bBox.width;
				l_array.push_back(l_ball);
			}
		}
	}

	return l_array;
}



/// COLOR FILTERING FUNCTION
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



// DRAW BALLS' AND SEARCH RECTANGLE
void Camera::final_image_showing()
{  
      // BALLS' RECTANGLES
	Point center;
	center.x=320;
	center.y=240;
	
  // SEARCH RECTANGLE
	for(size_t i = 0;i<m_robot_array.size();++i)
	{
		if(m_robot_array[i].pose_setted == 3 )
			rectangle(m_stream_video,m_robot_array[i].pose_rect,cv::Scalar(0, 0, 0),1,8,0);
	}
	cv::circle(m_stream_video,center,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(m_stream_video,center,10,cv::Scalar(0, 0, 0),1,8,0);
	createTrackbar("iFOVx",SENSOR_CV_WINDOW+m_camera_name,&m_focal_angle_x,255,0,0);
	createTrackbar("iFOVy",SENSOR_CV_WINDOW+m_camera_name,&m_focal_angle_y,255,0,0);
	cv::imshow(SENSOR_CV_WINDOW+m_camera_name,m_stream_video);
}



// GET THE CLASS ARRAY
std::vector<ball_position> Camera::get_blue_array()
{
	Lock l_lock(m_mutex);
	return m_blue_circles_W;
}



// GET THE CLASS ARRAY
std::vector<ball_position> Camera::get_green_array()
{
	Lock l_lock(m_mutex);
	return m_green_circles_W;
}




// GET THE CLASS ARRAY
std::vector<ball_position> Camera::get_red_array()
{
	Lock l_lock(m_mutex);
	return m_red_circles_W;
}




// GET IMAGE
Mat Camera::get_stream_video()
{
	Lock l_lock(m_mutex);
	return m_stream_video;
}



// GIVE THE CLASS ARRAY
std::vector<ball_position> Camera::get_yellow_array()
{
	Lock l_lock(m_mutex);
	return m_yellow_circles_W;
}

string Camera::get_name()
{
  Lock l_lock(m_mutex);
  return m_camera_name;
}

// FEEDBACK OF EKF NODE 
void Camera::pose_feedback(const nav_msgs::Odometry::ConstPtr& msg)
{
	std::vector<ball_position> corners_pixel,corners_cm_w;
	ball_position corn;
	int robot_number;
	float x_min,x_max,y_min,y_max;
	ball_position robot_position_cm,robot_position_pixel;
	ball_position robot_position_cm_W;
	std::string l_robot_name = msg->child_frame_id;
	l_robot_name = l_robot_name.substr(0,l_robot_name.find("/"));
	robot_position_cm_W.x = msg->pose.pose.position.x*100;// m to cm
	robot_position_cm_W.y = msg->pose.pose.position.y*100;// m to cm
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
	x_min = min(corners_cm_w.at(0).x,corners_cm_w.at(1).x);
	x_max = max(corners_cm_w.at(0).x,corners_cm_w.at(1).x);
	y_min = min(corners_cm_w.at(0).y,corners_cm_w.at(1).y);
	y_max = max(corners_cm_w.at(0).y,corners_cm_w.at(1).y);
	for(size_t k=2;k<corners_cm_w.size();++k)
	{
		x_min = min(x_min,corners_cm_w.at(k).x);
		x_max = max(x_max,corners_cm_w.at(k).x);
		y_min = min(y_min,corners_cm_w.at(k).y);
		y_max = max(y_max,corners_cm_w.at(k).y);
	}

	Lock l_lock(m_mutex);
	cv::Rect new_rect;
	cv::Rect field_vision;
	
	if (robot_position_cm_W.x<x_max && 
	    robot_position_cm_W.x>x_min && 
	    robot_position_cm_W.y<y_max && 
	    robot_position_cm_W.y>y_min )
	{
		int to_update = -1;
		robot_position_pixel = W_to_cam(robot_position_cm_W);
		for(size_t j = 0; j<m_robot_array.size(); ++j)
		{
			if(l_robot_name == m_robot_array[j].name)
			{
				to_update = j;
			}
		}
	    
		if(to_update >=0)
		{
			cv::Point l_tl,l_br;
			if (robot_position_pixel.y>240)
			{
				robot_position_pixel.height = 250;
				robot_position_pixel.width = 250;
 				l_tl.x = robot_position_pixel.x-robot_position_pixel.height/2;
 				l_tl.y = robot_position_pixel.y-robot_position_pixel.width/2;
				l_br.x = l_tl.x+robot_position_pixel.height;
				l_br.y = l_tl.y+robot_position_pixel.width;
 				new_rect.x = l_tl.x;
 				new_rect.y = l_tl.y;
				new_rect.height = robot_position_pixel.height;
				new_rect.width = robot_position_pixel.width;
				m_robot_array[to_update].pose_rect = new_rect;
			}else{
				robot_position_pixel.height = 175;
				robot_position_pixel.width = 175;
				l_tl.x = robot_position_pixel.x-robot_position_pixel.height/2;
 				l_tl.y = robot_position_pixel.y-robot_position_pixel.width/2;
				l_br.x = l_tl.x+robot_position_pixel.height;
				l_br.y = l_tl.y+robot_position_pixel.width;
 				new_rect.x = l_tl.x;
 				new_rect.y = l_tl.y;
				new_rect.height = robot_position_pixel.height;
				new_rect.width = robot_position_pixel.width;
				m_robot_array[to_update].pose_rect = new_rect;
			}
		}
	}else{
		int to_erase = -1;
		for(size_t j = 0; j<m_robot_array.size(); ++j)
		{
			if(l_robot_name == m_robot_array[j].name)
			{
				to_erase = j;
			}
		}
		if(to_erase >= 0)
		{
			new_rect.x = 320;
			new_rect.y = 240;
			new_rect.height = 0;
			new_rect.width = 0;
		}
		m_robot_array[to_erase].pose_rect = new_rect;
	} 
}

///
void Camera::robot_topic_pose_subscribe(RobotConfiguration robot_pose)
{
	ros::Subscriber pose_sub = m_node.subscribe<nav_msgs::Odometry>("/" + robot_pose.name + "/localizer/odometry/final", 10, &Camera::pose_feedback, this);
	m_robot_feedback_pose_sub.push_back(pose_sub);
        ROS_INFO("io sono %s ,in cam %s lui l'ho visto in %s",robot_pose.name.c_str(),m_camera_name.c_str(), robot_pose.cam_name.c_str());
	if( robot_pose.cam_name != m_camera_name )
	{
	  robot_pose.pose_rect.width = 0;
	  robot_pose.pose_rect.height= 0;
	}
	m_robot_array.push_back(robot_pose);
}

/// BALL SEARCHING 
void Camera::search_ball_pos()
{ 
	cv::Mat withCircle_blue,withCircle_green,withCircle_red,withCircle_yellow,l_bg,l_ry;
	cv::Mat l_only_blue, l_only_green, l_only_red, l_only_yellow;
	//withCircle_blue=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	withCircle_green=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	withCircle_red=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	withCircle_yellow=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	l_only_blue=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	l_only_green=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	l_only_red=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	l_only_yellow=Mat::zeros(m_stream_video.rows,m_stream_video.cols, m_stream_video.type());
	std::vector<ball_position> l_blue_circles,l_green_circles,l_red_circles,l_yellow_circles;
	m_stream_video.copyTo(withCircle_blue);
	  
	// THRESHOLDED IMAGES
	thresholded_images_settings();
	
	// FILTERING
	filtering(m_stream_video,l_only_blue,m_lb_b,m_ub_b,m_dim_kernel_blue+1);  
	filtering(m_stream_video,l_only_green,m_lb_g,m_ub_g,m_dim_kernel_green+1);  
	cv::Mat l_only_lower_red, l_only_upper_red;
	filtering(m_stream_video,l_only_lower_red,m_lower_lb_r,m_lower_ub_r,m_dim_kernel_red+1);  
	filtering(m_stream_video,l_only_upper_red,m_upper_lb_r,m_upper_ub_r,m_dim_kernel_red+1);  
	cv::addWeighted(l_only_lower_red,1.0,l_only_upper_red,1.0,0.0,l_only_red);
	filtering(m_stream_video,l_only_yellow,m_lb_y,m_ub_y,m_dim_kernel_yellow+1);  
		
	//  CHARGE ARRAY WITH FIND BALLS 
	l_blue_circles = charge_array(l_only_blue);
 	l_green_circles = charge_array(l_only_green);
 	l_red_circles = charge_array(l_only_red);
 	l_yellow_circles = charge_array(l_only_yellow);

	
      //     FROM CAM (pixel) TO WORLD (cm)
// 	Lock l_lock(m_mutex);
	m_blue_circles_W = cam_to_W(l_blue_circles);
 	m_green_circles_W = cam_to_W(l_green_circles);
 	m_red_circles_W = cam_to_W(l_red_circles);
	m_yellow_circles_W = cam_to_W(l_yellow_circles);
	
	// SHOW IMAGE
	final_image_showing();

}

/// NEW ROBOT TOPIC SUBSCRIBE 
void Camera::subscribe()
{
	cv::namedWindow(SENSOR_CV_WINDOW+m_camera_name);
	m_image_sub = m_it.subscribe(m_topic_name, 1, &Camera::video_acquisition, this, image_transport::TransportHints("raw")); 
}



// HSV TRACKBAR FOREACH COLOR 
void Camera::thresholded_images_settings()
{
	// Blue Ball HSV values 
	namedWindow(BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
	createTrackbar("H lower",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_b[0],180,0,0);
	createTrackbar("S lower",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_b[1],255,0,0);
	createTrackbar("V lower",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_b[2],255,0,0);
	createTrackbar("H upper",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_b[0],180,0,0);
	createTrackbar("S upper",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_b[1],255,0,0);
	createTrackbar("V upper",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_b[2],255,0,0);
	createTrackbar("Kernel size",BLUE_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_dim_kernel_blue,20,0,0);
	

	//      Green Ball HSV values
	namedWindow(GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
	createTrackbar("H lower",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_g[0],180,0,0);
	createTrackbar("S lower",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_g[1],255,0,0);
	createTrackbar("V lower",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_g[2],255,0,0);
	createTrackbar("H upper",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_g[0],180,0,0);
	createTrackbar("S upper",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_g[1],255,0,0);
	createTrackbar("V upper",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_g[2],255,0,0);
	createTrackbar("Kernel size",GREEN_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_dim_kernel_green,20,0,0);


	//      Red Ball HSV values (H had *0.5 scale factor)
	namedWindow(RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
	createTrackbar("H lower (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lower_lb_r[0],180,0,0);
	createTrackbar("S lower (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lower_lb_r[1],255,0,0);
	createTrackbar("V lower (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lower_lb_r[2],255,0,0);
	createTrackbar("H upper (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lower_ub_r[0],180,0,0);
	createTrackbar("S upper (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lower_ub_r[1],255,0,0);
	createTrackbar("V upper (lower red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lower_ub_r[2],255,0,0);
	createTrackbar("H lower (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_upper_lb_r[0],180,0,0);
	createTrackbar("S lower (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_upper_lb_r[1],255,0,0);
	createTrackbar("V lower (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_upper_lb_r[2],255,0,0);
	createTrackbar("H upper (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_upper_ub_r[0],180,0,0);
	createTrackbar("S upper (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_upper_ub_r[1],255,0,0);
	createTrackbar("V upper (upper red)",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_upper_ub_r[2],255,0,0);
	createTrackbar("Kernel size",RED_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_dim_kernel_red,20,0,0);
	    
	//      Yellow Ball HSV values
	namedWindow(YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name);
	createTrackbar("H lower",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_y[0],180,0,0);
	createTrackbar("S lower",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_y[1],255,0,0);
	createTrackbar("V lower",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_lb_y[2],255,0,0);
	createTrackbar("H upper",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_y[0],180,0,0);
	createTrackbar("S upper",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_y[1],255,0,0);
	createTrackbar("V upper",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_ub_y[2],255,0,0);
	createTrackbar("Kernel size",YELLOW_THRESHOLD_WINDOWS+"calibration"+m_camera_name,&m_dim_kernel_yellow,20,0,0);
	
	// //      THRESHOLDED IMG
	//      imshow(BLUE_THRESHOLD_WINDOWS+m_camera_name,m_only_blue);
	//      imshow(GREEN_THRESHOLD_WINDOWS+m_camera_name,m_only_green);
	//      imshow(RED_THRESHOLD_WINDOWS+m_camera_name,m_only_red);
	//      imshow(YELLOW_THRESHOLD_WINDOWS+m_camera_name,m_only_yellow);  
}



/// GET IMAGE FROM CAMERA TOPIC
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
	Lock l_lock(m_mutex);
	m_stream_video = cv_ptr->image.clone();
	// Update GUI Window
	cv::waitKey(1);
	search_ball_pos();
}

////// NEW 
ball_position Camera::W_to_cam(ball_position& pos_in)
{
	ball_position pos_cam_pixel;
	float pos_cam_cm[3],pos_world[3],o01[3];
	float iFOVx = (m_focal_angle_x*M_PI/180)/640;
	float iFOVy = (m_focal_angle_y*M_PI/180)/480;
	float azimuth,elevation;
	float psi1,psi2,rx;
	float distance_from_center_x,distance_from_center_y;
	float pitch = -atan(m_R/m_zCamera)+M_PI/2;
	float R_z[3][3],R_x[3][3],Rtot[3][3];
	float x_SR_centered,y_SR_centered,x_roll_corrected,y_roll_corrected;
	o01[0] = m_xCamera;
	o01[1] = m_yCamera;
	float ipotenuse = sqrt(pow(m_zCamera,2)+pow(m_R,2));
	// TRASLATION
	pos_world[0] = pos_in.x-o01[0];
	pos_world[1] = pos_in.y-o01[1];
	pos_world[2] = 0;
	// ROTATION
	// Rz * Rx ( used the transposed one )
	Rtot[1][1] = cos(m_omegaz);
	Rtot[1][2] = -cos(m_gammax)*sin(m_omegaz);
	Rtot[1][3] = sin(m_gammax)*sin(m_omegaz);
	Rtot[2][1] = sin(m_omegaz);
	Rtot[2][2] = cos(m_gammax)*cos(m_omegaz);
	Rtot[2][3] = -sin(m_gammax)*cos(m_omegaz);
	Rtot[3][1] = 0;
	Rtot[3][2] = sin(m_gammax);
	Rtot[3][3] = cos(m_gammax);
	pos_cam_cm[0] = Rtot[1][1]*pos_world[0]+Rtot[2][1]*pos_world[1]+Rtot[3][1]*pos_world[2];
	pos_cam_cm[1] = Rtot[1][2]*pos_world[0]+Rtot[2][2]*pos_world[1]+Rtot[3][2]*pos_world[2];
	pos_cam_cm[2] = Rtot[1][3]*pos_world[0]+Rtot[2][3]*pos_world[1]+Rtot[3][3]*pos_world[2];
	// SR under camera
	distance_from_center_x = pos_cam_cm[0];
	distance_from_center_y = m_R+pos_cam_cm[1];
	// h robot correction 
	psi2 = atan((m_R-distance_from_center_y)/m_zCamera);
	distance_from_center_y = distance_from_center_y-m_h_robot*tan(psi2);
	// y correction 
	elevation = atan2(distance_from_center_y*sin(M_PI-pitch),ipotenuse+distance_from_center_y*cos(M_PI-pitch));
	y_roll_corrected = elevation/iFOVy;
	// x correction
	rx = (sin(pitch)/sin(M_PI-pitch-elevation))*ipotenuse;
	rx = rx-sqrt(pow(m_h_robot,2)+pow(m_h_robot*tan(psi2),2));
	azimuth = atan2(distance_from_center_x,rx);
	x_roll_corrected = azimuth/iFOVx;
		
	  
	x_SR_centered = x_roll_corrected*cos(m_roll)+y_roll_corrected*sin(m_roll);
	y_SR_centered = -x_roll_corrected*sin(m_roll)+y_roll_corrected*cos(m_roll);
	pos_cam_pixel.x = x_SR_centered+320.5;
	pos_cam_pixel.y = y_SR_centered+240.5;
	pos_cam_pixel.height = 2*g_ball_radius;
	pos_cam_pixel.width = 2*g_ball_radius;
	return pos_cam_pixel;
}
