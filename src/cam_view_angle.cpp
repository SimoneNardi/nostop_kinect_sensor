#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


std::vector<cv::Point2f> vertex;
const char* camera_name = "camera";
bool ready,callback_on=false;

// CAMERA VALUES
static float distance = 40;
static float x_edge_cm = 29.5;
static float y_edge_cm = 21;

// POINT FROM LEFT TO RIGHT AND FROM TOP TO BOTTOM (CLOCKWISE)
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) { 
		cv::Point2f xy;
		xy.x = x;
		xy.y = y;
		vertex.push_back(xy);
		int number;
		number = vertex.size();
		ROS_INFO("Vertex %d inserted",number+1);
		if(number == 4)
		{
		  ready = true;
		}else{
		  ready = false;
		  }
		callback_on = false;
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
	cv::Point center,right;
	center.x=320.5;
	center.y=240.5;
	right.x = 637;
	right.y = 240.5;
	cv::circle(video_image,right,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::circle(video_image,center,2,cv::Scalar(0, 0, 255),-1,8,0);
	cv::line(video_image,center,right,cv::Scalar(0,255,0),0,8,0);
	cv::imshow(camera_name,video_image);
	if(!callback_on)
	  cvSetMouseCallback(camera_name,mouse_callback,NULL);
}


int main(int argc, char **argv)
{
	ROS_INFO("Calibration camera view angle: ON");
	ros::init(argc, argv,"camera");
	ros::NodeHandle angle_view("~");
	image_transport::ImageTransport it(angle_view);
	image_transport::Subscriber subscriber;
	ros::Publisher publisher;
	std::string image_topic;
	angle_view.getParam("image_topic",image_topic);
	publisher = angle_view.advertise<std_msgs::Float64MultiArray>("/view_angle_founded",10);
	subscriber = it.subscribe(image_topic, 10, &subscriber_callback,image_transport::TransportHints("raw"));
	while(ros::ok())
	{
		ros::spinOnce();
		if(ready)
		{
			cv::Point2f A,B,C,D;
			float x_edge_pixel,y_edge_pixel;
			A = vertex.at(0);
			B = vertex.at(1);
			C = vertex.at(2);
			D = vertex.at(3);
			x_edge_pixel = B.x-A.x;
			y_edge_pixel = C.y-B.y;
			float alpha = 2*atan((x_edge_cm/2)/distance);
			float beta = 2*atan((y_edge_cm/2)/distance);
			float iFOVx = alpha/x_edge_pixel;
			float iFOVy = beta/y_edge_pixel;
			std_msgs::Float64MultiArray message;
			message.data.resize(2);
			message.data[0] = (iFOVx*180/M_PI)*640;
			message.data[1] = (iFOVy*180/M_PI)*480;
			publisher.publish<std_msgs::Float64MultiArray>(message);
			vertex.clear();
			ready = false;
		}
	}
}