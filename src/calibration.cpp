#include "ros/ros.h"
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <nostop_kinect_sensor/Camera_calibrationConfig.h>
#include <nostop_kinect_sensor/Camera_data_msg.h>
#include <nostop_kinect_sensor/Camera_data_srv.h>
#include <iostream>
#include <fstream>
#include <cv.h>
#include <stdlib.h>

#define message_size 12 
cv::Point2f xy;
std::vector<cv::Point2f> vertex;
cv::Point2f A_toimage;
std_msgs::Float64MultiArray message;
bool to_publish,rool_cal_window_on,callback_on;
std::string cam_name;
int image_width,image_height;
 

void mouse_callback(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDBLCLK) 
	{ 
		xy.x = x;
		xy.y = y;
		vertex.push_back(xy);
		callback_on = false;
	}
}


void subscriber_callback(const sensor_msgs::ImageConstPtr &msg)
{
	std::string window_name = cam_name+" roll calibration";
        cv::namedWindow(window_name);
	if(rool_cal_window_on)
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
		image_width = video_image.cols;
		image_height = video_image.rows;
		cv::waitKey(3);
		cv::Point center,up,right,down,left;
		center.x=image_width/2;
		center.y=image_height/2;
		up.x = image_width/2;
		up.y = 2;
		right.x = image_width-3;
		right.y = image_height/2;
		down.x = image_width/2;
		down.y = image_height-3;
		left.x = 3;
		left.y = image_height/2;
		cv::circle(video_image,down,2,cv::Scalar(0, 0, 255),-1,8,0);
		cv::circle(video_image,left,2,cv::Scalar(0, 0, 255),-1,8,0);
		cv::circle(video_image,up,2,cv::Scalar(0, 0, 255),-1,8,0);
		cv::circle(video_image,right,2,cv::Scalar(0, 0, 255),-1,8,0);
		cv::circle(video_image,center,2,cv::Scalar(0, 0, 255),-1,8,0);
		cv::circle(video_image,center,10,cv::Scalar(0, 0, 0),1,8,0);
		cv::line(video_image,center,right,cv::Scalar(0,255,0),0,8,0);
		if(A_toimage.x > image_width/2)
		{
			cv::line(video_image,center,A_toimage,cv::Scalar(0,255,0),0,8,0);
		} else {
			cv::line(video_image,center,A_toimage,cv::Scalar(0,0,255),0,8,0);
			cv::Point2f A_symmetric,center_symmetric;
			A_symmetric.x = image_width-A_toimage.x;
			A_symmetric.y = image_height-A_toimage.y;
			center_symmetric.x = image_width/2;
			center_symmetric.y = A_toimage.y;
			cv::line(video_image,center,A_symmetric,cv::Scalar(0,255,0),0,8,0);
			cv::line(video_image,A_toimage,center_symmetric,cv::Scalar(0,0,255),0,8,0);
		}
		cv::imshow(window_name,video_image);
		if(!callback_on)
		{
			cvSetMouseCallback(window_name.c_str(),mouse_callback,NULL);
			callback_on = true;
		}
		
		if (vertex.size()==1)
		{ 
			to_publish = true;
			cv::Point2f A;
			A = vertex[0];
			A_toimage = A;
			A.y = A.y-image_height/2;
			A.x = A.x-image_width/2;
			float roll_angle = atan(A.y/A.x)*180/M_PI;
			message.data[0] = roll_angle;
			vertex.clear();
		}
	}else{
		cv::destroyWindow(window_name);
		callback_on = false;
	}
}



float rot_Z(float xC,float yC,float xP,float yP)
{
	float omega;
	omega = atan2(yP-yC,xP-xC);
	return omega;
}

bool from_file = false;
float readed_cfg[message_size];
void calibration_callback(nostop_kinect_sensor::Camera_calibrationConfig  &config, uint32_t level) 
 {
	float R,xC,yC,zC,xW,yW,xP,yP,omega_z,gam,h_robot;
	int gps_time,max_area,min_area;
	float HSV_calibration_on,roll_calibration;
// 	if(from_file)
// 	{
// 	  config.R_distance=message.data[1];
// 	}
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
// 	gam = config.gamma_xC;
	gam = 180;
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
	if(config.Roll_cal_window)
	  rool_cal_window_on = true;
	else
	  rool_cal_window_on = false;
	
	//TEST
	message.data[10] = config.MinArea;
	message.data[11] = config.MaxArea;
	
	to_publish = true;
}


int main(int argc, char **argv)
{
	std::string calibration_topic,image_topic,user_name;
	int iFOVx,iFOVy;
	ros::init(argc, argv, "calibration");
	ros::NodeHandle calibrator("~");
	ros::Publisher calibrator_pub;
	ros::ServiceClient camera_in_pub;
	ROS_INFO("Calibration %s : ON",cam_name.c_str());
	message.data.resize(message_size);
	
	// initialization
	rool_cal_window_on = false;
	to_publish = false;
	callback_on = false;;

	calibrator.getParam("camera_name",cam_name);
	calibrator.getParam("image_topic",image_topic);
	calibrator.getParam("calibration_topic_name",calibration_topic);
	calibrator.getParam("iFOVx",iFOVx);
	calibrator.getParam("iFOVy",iFOVy);
	calibrator.getParam("user_name",user_name);
	calibrator.getParam("image_width",image_width);
        calibrator.getParam("image_height",image_height);

	image_transport::ImageTransport it(calibrator);
	image_transport::Subscriber subscriber;
	subscriber = it.subscribe(image_topic.c_str(),3, &subscriber_callback,image_transport::TransportHints("raw"));
	calibrator_pub = calibrator.advertise<std_msgs::Float64MultiArray>(calibration_topic,10);
	camera_in_pub = calibrator.serviceClient<nostop_kinect_sensor::Camera_data_srv>("/camera_in");

	dynamic_reconfigure::Server<nostop_kinect_sensor::Camera_calibrationConfig> camera_calibration;
	dynamic_reconfigure::Server<nostop_kinect_sensor::Camera_calibrationConfig>::CallbackType callback;
	callback = boost::bind(&calibration_callback,_1,_2);
	camera_calibration.setCallback(callback);
	nostop_kinect_sensor::Camera_data_srv camera_in;
	camera_in.request.calibration_topic = calibration_topic;
	camera_in.request.ifovx = iFOVx;
	camera_in.request.ifovy = iFOVy;
	camera_in.request.name = cam_name;
	camera_in.request.topic_name = image_topic;
	bool camera_in_setted = false;
	while(!camera_in_setted)
	{
		if(camera_in_pub.call<nostop_kinect_sensor::Camera_data_srv>(camera_in))
			camera_in_setted = true;
		ros::spinOnce();
	}
	//TODO
// 	std::ofstream outputFile;
// 	std::ifstream inputFile;
// 	bool file_readed;
// // 	std::string file_name;user_name = "niko";cam_name="philips";
// 	file_name = "/home/"+user_name+"/catkin_ws/src/nostop/nostop_kinect_sensor/"+cam_name+"_calibration_file.txt";
// 	inputFile.open(file_name.c_str());
// 	if(inputFile.is_open())
// 	{
// 		int count;
// 		char output[100];
// 		std::cout<<"Calibration file founded in: "+ file_name.substr(0,file_name.find_last_of("/"))<<std::endl;
// 
// 		while(count<message_size)
// 		{
// 			inputFile >> output;
// // 			std::cout << output<<std::endl;
// 			float readed = atof(output);
// 			readed_cfg[count] = readed;
// 			count+=1;
// 		}
// 		from_file =  true;
// 		to_publish = true;
// 	}else{
// 		std::cout<<"Calibration file not founded in: "+file_name.substr(0,file_name.find_last_of("/"))<<std::endl;
// 	}
// 	inputFile.close();
	A_toimage.x=image_width/2;
	A_toimage.y=image_height/2;
	while(ros::ok)
	{	
		ros::spinOnce();
		if(to_publish)
		{  
		//TODO
// 			outputFile.open(file_name.c_str());
// 			for(size_t i=0;i<message_size;++i)
// 			{
// 				float local;
// 				local = message.data.at(i);
// 				outputFile << local <<std::endl;
// 			}

// 			outputFile.close();
// 			if(from_file)
// 			{
// 				for(size_t i=0;i<message_size;++i)
// 				{
// 					message.data.at(i) = readed_cfg[i];
// 				}
// 				from_file = false;
// 			}
			calibrator_pub.publish(message);
			ROS_INFO("%s published calibration values",cam_name.c_str());
			to_publish=false;
		}
	}
	return 0;
}
