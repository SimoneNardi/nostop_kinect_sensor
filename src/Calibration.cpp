#include "ros/ros.h"
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64.h>

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

    cv::Mat m_stream_video = cv_ptr->image.clone();
    cv::imshow("test",m_stream_video);
    cv::waitKey(3);
}


int main(int argc, char **argv)
{
	ROS_INFO("Calibration : ON");
	
	ros::init(argc, argv, "calibrator"); 
      
	ros::NodeHandle calibrator;
	ros::Publisher Roll_pub;
	image_transport::ImageTransport it(calibrator);
	image_transport::Subscriber subscriber;
	cv::namedWindow("test");
	subscriber = it.subscribe("TODO", 1, &subscriber_callback,image_transport::TransportHints("raw"));
	Roll_pub = calibrator.advertise<std_msgs::Float64>("TODO",1000);
	if (vertex.size()==2)
	{
	  vertex.clear();
          Roll_pub.publish<std_msgs::Float64>(roll_value);
	}
	ros::spin();
     
	return 0;
}
