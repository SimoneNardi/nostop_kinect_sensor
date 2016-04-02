#include "ros/ros.h"
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
 
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <nostop_kinect_sensor/Camera_calibrationConfig.h>
#include <cv.h>
#include <stdlib.h>
cv::Point2f xy;
std::vector<cv::Point2f> vertex;
cv::Point2f A_toimage;
std_msgs::Float64MultiArray message;
bool to_publish,callback_on = false;
std::string cam_name;
 
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) { 
		xy.x = x;
		xy.y = y;
		vertex.push_back(xy);
	}
	callback_on = true;
}


void subscriber_callback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
	  if (sensor_msgs::image_encodings::isColor(msg->encoding))
		  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	  else
		  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
	}catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat video_image = cv_ptr->image.clone();
	cv::waitKey(3);
	cv::Point center,up,right,down,left;
	center.x=320.5;
	center.y=240.5;
	up.x = 320.5;
	up.y = 2;
	right.x = 637;
	right.y = 240.5;
	down.x = 320.5;
	down.y = 477;
	left.x = 3;
	left.y = 240.5;
	cv::circle(video_image,down,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(video_image,left,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(video_image,up,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(video_image,right,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(video_image,center,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(video_image,center,10,cv::Scalar(0, 0, 0),1,8,0);
	cv::line(video_image,center,right,cv::Scalar(0,255,0),0,8,0);
	if(A_toimage.x > 320)
	{
		cv::line(video_image,center,A_toimage,cv::Scalar(0,255,0),0,8,0);
	} else {
		cv::line(video_image,center,A_toimage,cv::Scalar(0,0,255),0,8,0);
		cv::Point2f A_symmetric,center_symmetric;
		A_symmetric.x = 640-A_toimage.x;
		A_symmetric.y = 480-A_toimage.y;
		center_symmetric.x = 320;
		center_symmetric.y = A_toimage.y;
		cv::line(video_image,center,A_symmetric,cv::Scalar(0,255,0),0,8,0);
		cv::line(video_image,A_toimage,center_symmetric,cv::Scalar(0,0,255),0,8,0);
	}
	cv::imshow(cam_name.c_str(),video_image);
	if(!callback_on)
	  cvSetMouseCallback(cam_name.c_str(),mouse_callback,NULL);
	
	if (vertex.size()==1)
	{ 
		to_publish = true;
		cv::Point2f A;
		A = vertex[0];
		A_toimage = A;
		A.y = A.y-240;
		A.x = A.x-320;
		float roll_angle = atan(A.y/A.x)*180/M_PI;
		message.data[0] = roll_angle;
		vertex.clear();
	}
}



float rot_Z(float xC,float yC,float xP,float yP)
{
	float omega;
	omega = atan2(yP-yC,xP-xC);
	return omega;
}


void calibration_callback(nostop_kinect_sensor::Camera_calibrationConfig  &config, uint32_t level) 
 {
	float R,xC,yC,zC,xW,yW,xP,yP,omega_z,gam,h_robot;
	int gps_time;
	float HSV_calibration_on;
	R = config.R_distance;
	message.data[1] = R;
	xC = config.W_xC;
	message.data[2] = xC;
	yC = config.W_yC;
	message.data[3] = yC;
	zC = config.W_zC;
	message.data[4] = zC;
	xP = config.W_xAxesP;
	yP = config.W_yAxesP;
	omega_z = rot_Z(xC,yC,xP,yP);
	message.data[5] = omega_z;
	gam = config.gamma_xC;  
	message.data[6] = gam;
	h_robot = config.h_robot;
	message.data[7] = h_robot;
	gps_time = config.lost_gps_time;
	message.data[8] = (float) gps_time;
	if(config.HSV_calibration)
	  HSV_calibration_on = 1;
	else
	  HSV_calibration_on = 0;
	message.data[9] = HSV_calibration_on;
	to_publish = true;
}


int main(int argc, char **argv)
{
	std::string calibration_topic,image_topic;
	ros::init(argc, argv, "calibration");
	ros::NodeHandle calibrator("~");
	ros::Publisher calibrator_pub;
	ROS_INFO("Calibration %s : ON",cam_name.c_str());
	message.data.resize(10);
	calibrator.getParam("camera_name",cam_name);
	calibrator.getParam("image_topic",image_topic);
	calibrator.getParam("calibration_topic_name",calibration_topic);
	A_toimage.x=320.5;
	A_toimage.y=240.5;
	image_transport::ImageTransport it(calibrator);
	image_transport::Subscriber subscriber;
	subscriber = it.subscribe(image_topic.c_str(),3, &subscriber_callback,image_transport::TransportHints("raw"));
	calibrator_pub = calibrator.advertise<std_msgs::Float64MultiArray>(calibration_topic,10);
	dynamic_reconfigure::Server<nostop_kinect_sensor::Camera_calibrationConfig> camera_calibration;
	dynamic_reconfigure::Server<nostop_kinect_sensor::Camera_calibrationConfig>::CallbackType callback;
	callback = boost::bind(&calibration_callback,_1,_2);
	camera_calibration.setCallback(callback);
	
	while (ros::ok())
	{
		if(to_publish)
		{  
			calibrator_pub.publish(message);
			ROS_INFO("%s published calibration values",cam_name.c_str());
			to_publish=false;
		}
		ros::spinOnce();
	}
	return 0;
}
