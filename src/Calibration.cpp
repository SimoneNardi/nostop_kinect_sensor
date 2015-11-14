#include "ros/ros.h"
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <nostop_kinect_sensor/R_valueConfig.h>

cv::Point2f xy;
std::vector<cv::Point2f> vertex;
std_msgs::Float64MultiArray message;
bool to_publish;
float R=1;
 
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	//This is called every time a mouse event occurs in the window
	if (event == CV_EVENT_LBUTTONDBLCLK) { //This is executed when the left mouse button is clicked
		//Co-ordinates of the left click are assigned to global variables and flag is set to 1
		xy.x = x;
		xy.y = y;
		vertex.push_back(xy);
	}
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
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

    cv::Mat video_image = cv_ptr->image.clone();
    cvNamedWindow("Frame",CV_WINDOW_AUTOSIZE); //Window is created for image of each frame
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
    cv::imshow("Frame",video_image);
    cvSetMouseCallback("Frame",mouse_callback,NULL);	
      if (vertex.size()==1)
	    { 
	         to_publish = true;
	         cv::Point2f A;
		 A = vertex[0];
		 A.y = A.y-240;
		 A.x = A.x-320;
                 float roll_angle = atan(A.y/A.x)*180/M_PI;
                 message.data[0] = roll_angle;
		 vertex.clear();
	    }

}

void R_reconfigure(nostop_kinect_sensor::R_valueConfig  &config, uint32_t level) 
 {
  R = config.R_distance;
  to_publish = true;
}
int main(int argc, char *argv[])
{
	ROS_INFO("Calibration : ON");
	ros::init(argc, argv, "calibrator");
	message.data.resize(2);
	ros::NodeHandle calibrator;
	ros::Publisher Roll_R_pub;
	image_transport::ImageTransport it(calibrator);
	image_transport::Subscriber subscriber;
	subscriber = it.subscribe(argv[1], 1, &subscriber_callback,image_transport::TransportHints("raw"));
	Roll_R_pub = calibrator.advertise<std_msgs::Float64MultiArray>(argv[2],1000);
	
	dynamic_reconfigure::Server<nostop_kinect_sensor::R_valueConfig> R_camera;
	dynamic_reconfigure::Server<nostop_kinect_sensor::R_valueConfig>::CallbackType f;
	f = boost::bind(&R_reconfigure,_1,_2);
	R_camera.setCallback(f);
	
	while (ros::ok())
	{
	    if(to_publish)
	      {	  
		 message.data[1] = R;
                 Roll_R_pub.publish(message);
	         ROS_INFO("Value published");
	         to_publish=false;
	      }
	      ros::spinOnce();
	}
	return 0;
}
