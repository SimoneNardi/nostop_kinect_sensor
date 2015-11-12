#include "ros/ros.h"
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

cv::Point2f xy;
std::vector<cv::Point2f> vertex;
std_msgs::Float64 message;
bool to_publish;
 
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
    cv::imshow("Frame",video_image);
    cvSetMouseCallback("Frame",mouse_callback,NULL);	
      if (vertex.size()==2)
	    { 
	         to_publish = true;
	         cv::Point2f A,B;
                 if (vertex[0].x > vertex[1].x)
                   {
                      A = vertex[1];
                      B = vertex[0];
                   }else{
                           A = vertex[0];
                           B = vertex[1];
                        }
                 float iFOVy= 31.5/480;
                 float roll_angle = (A.y-B.y)*iFOVy;
                 std_msgs::Float64 roll;
                 message.data=roll_angle;
		 vertex.clear();
	    }
    
}

int main(int argc, char *argv[])
{
	ROS_INFO("Calibration : ON");
	ros::init(argc, argv, "calibrator");
	ros::NodeHandle calibrator;
	ros::Publisher Roll_pub;
	image_transport::ImageTransport it(calibrator);
	image_transport::Subscriber subscriber;
	subscriber = it.subscribe(argv[argc-1], 1, &subscriber_callback,image_transport::TransportHints("raw"));
	Roll_pub = calibrator.advertise<std_msgs::Float64>("/test",1000);
	while (ros::ok())
	{
	    ros::spinOnce();
	    if(to_publish)
	      {	  
                 Roll_pub.publish(message);
	         ROS_INFO("Value published");
	         to_publish=false;
	      }
	}
	return 0;
}
