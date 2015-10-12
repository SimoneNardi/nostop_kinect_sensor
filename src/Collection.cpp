#include "Collection.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include "ros/ros.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace cv;


static const std::string FOREGROUND_CV_WINDOW = "Foreground photo Window (with circles)";
static const std::string SENSOR_CV_WINDOW = "Sensor view Window";
static const std::string FILTERED_CV_WINDOW = "Filtered view Window";
 
/////////////////////////////////////////////
Collection::Collection()
: m_available(false)
, m_data()
, m_it(m_node)
, m_foregroundFLAG(false)
, m_wait_time(140)
, m_count(0)
, m_dp(1)
, m_minDist(300)
, m_param1(15)
, m_param2(16)
, m_minR(15)
, m_maxR(30)
, m_max_red(255)
, m_min_red(0)
, m_max_green(255)
, m_min_green(0)
, m_max_blue(255)
, m_min_blue(0)
, m_thr(40)
, m_maxval(255)
{}

/////////////////////////////////////////////
Collection::~Collection()
{
	cv::destroyWindow(FOREGROUND_CV_WINDOW); //destroy the window with the name, "MySensorWindow"
	cv::destroyWindow(SENSOR_CV_WINDOW);
// 	cv::destroyWindow("Subtraction Image");
}

/////////////////////////////////////////////
void Collection::subscribe()
{
	ROS_INFO("Sensor: Collection subscribe!");
	
	cv::namedWindow(SENSOR_CV_WINDOW);
	cv::namedWindow(FOREGROUND_CV_WINDOW);
	m_image_pub = m_it.advertise("/sensor/output_video", 1);
	m_image_sub = m_it.subscribe("/camera/rgb/image_rect_color", 1, &Collection::getForeground, this, image_transport::TransportHints("raw"));
	m_image_sub_photo = m_it.subscribe("/sensor/output_video",1, &Collection::toPub, this, image_transport::TransportHints("raw"));
	
	while (!m_foregroundFLAG)
	{
	  ros::spinOnce();
	}
	
	m_foregroundFLAG = false;

	std::cout << "Sensor: ForeGround Collected!"<< std::endl << std::flush;

}

//////////////////////////////////////////
void Collection::getForeground(const sensor_msgs::ImageConstPtr& msg)
{
//   Lock l_lck(m_mutex);

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
  
  m_foreground = cv_ptr->image.clone();
  
  m_foregroundFLAG = true;
    
   if(m_foregroundFLAG)
   {
      // Update GUI Window
      cv::imshow(SENSOR_CV_WINDOW, m_foreground);
      cv::waitKey(3);
      m_image_pub.publish(msg);
      if (m_count == m_wait_time)
      {
	m_image_sub_photo.shutdown();
	m_image_pub.shutdown();
	m_count = m_count+1;
      } else {
	m_count = m_count+1;
      }
   }
     
}

//////////////////////////////////////////////////////////
void Collection::toPub(const sensor_msgs::ImageConstPtr& msg)
{
  //Lock l_lck(m_mutex);
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
  
  m_photo = cv_ptr->image.clone();
  m_photo_support = cv_ptr->image.clone();
  m_foregroundFLAG = true;
    
   if(m_foregroundFLAG)
   {
      // Update GUI Window
      cv::imshow(FOREGROUND_CV_WINDOW, m_photo);
      cv::waitKey(3);
   }	

}


/////////////////////////////////////////////////////
void Collection::searchCircles()
{	
	ROS_INFO("Searching init.");
	m_image_sub_circles = m_it.subscribe("/camera/rgb/image_rect_color", 1, &Collection::search_test, this, image_transport::TransportHints("raw"));
}

void Collection::search_test(const sensor_msgs::ImageConstPtr& msg)
{ 
  vector<cv::Vec3f> l_circles;
  cv::Mat l_first_filtered_image,l_second_filtered_image;
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  m_processed = cv_ptr->image.clone();
  
  // Filtering 
  l_first_filtered_image = abs(m_foreground-m_photo);
  cv::createTrackbar("Min red", FILTERED_CV_WINDOW, &m_min_red, 255);
  cv::createTrackbar("Max red", FILTERED_CV_WINDOW, &m_max_red, 255);
  cv::createTrackbar("Min green", FILTERED_CV_WINDOW, &m_min_green, 255);
  cv::createTrackbar("Max green", FILTERED_CV_WINDOW, &m_max_green, 255);
  cv::createTrackbar("Min blue", FILTERED_CV_WINDOW, &m_min_blue, 255);
  cv::createTrackbar("Max blue", FILTERED_CV_WINDOW, &m_max_blue, 255);
  cv::inRange(l_first_filtered_image,cv::Scalar(m_min_blue,m_min_green,m_min_red),Scalar(m_max_blue,m_max_green,m_max_red),l_second_filtered_image);
  cv::imshow("After First Filtering",l_first_filtered_image);
  cv::namedWindow(FILTERED_CV_WINDOW);
  cv::imshow(FILTERED_CV_WINDOW,l_second_filtered_image);
  
  
  // Find circles
  
  cv::HoughCircles(l_second_filtered_image,l_circles,CV_HOUGH_GRADIENT,m_dp,m_minDist,m_param1,m_param2,m_minR,m_maxR);
 
  cv::createTrackbar("Inverse ratio resolution", SENSOR_CV_WINDOW, &m_dp, 255);
  cv::createTrackbar("Min Dist between Centers", SENSOR_CV_WINDOW, &m_minDist, 255);
  cv::createTrackbar("Param 1", SENSOR_CV_WINDOW, &m_param1, 255);
  cv::createTrackbar("Param 2", SENSOR_CV_WINDOW, &m_param2, 255);
  cv::createTrackbar("Min rad", SENSOR_CV_WINDOW, &m_minR, 255);
  cv::createTrackbar("Max rad", SENSOR_CV_WINDOW, &m_maxR, 255);
//  cv::createTrackbar("Thr", SENSOR_CV_WINDOW, &l_thr, 255);
// cv::createTrackbar("Max Val", SENSOR_CV_WINDOW, &l_maxval, 255);	
  
  
  /// Draw the circles detected
  for( size_t i = 0; i < l_circles.size(); i++ )
  {
      cv::Point center(cvRound(l_circles[i][0]), cvRound(l_circles[i][1]));
      int radius = cvRound(l_circles[i][2]);
      // circle center
      cv::circle(m_photo_support, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      cv::circle( m_photo_support, center, radius, cv::Scalar(255,0,0), 3, 8, 0 );  
      cv::imshow(FOREGROUND_CV_WINDOW,m_photo_support);
   }
   
   
//   if(l_circles.size()==0)
//   { 
//     ROS_INFO("NOT FOUND");
//   }
//   else
//   { 
//     ROS_INFO("FOUND");
//   }
}

	
// enum ColorName
// {
//     red = 0,
//     blue,
//     green,
//     yellow
// };
//   
// /////////////////////////////////////////////
// void  computeColorRange(ColorName name, cv::Scalar &min, cv::Scalar &max)
// {
//     switch(name)
//     {
//       case ColorName::red:
// 	min = cv::Scalar(0,100,100);
// 	max = cv::Scalar(10,100,100);
// 	break;
//       case ColorName::blue:
// 	min = cv::Scalar(220,100,100);
// 	max = cv::Scalar(240,100,100);
// 	break;
//       case ColorName::yellow:
// 	min = cv::Scalar(60,100,100);
// 	max = cv::Scalar(70,100,100);
// 	break;
//       case ColorName::green:
// 	min = cv::Scalar(110,100,100);
// 	max = cv::Scalar(130,100,100);
//       default:
// 	break;
//     }
// }
// 
// /** @function Erosion */
// void Erosion( int erosion_elem, int erosion_size, cv::Mat const& src, cv::Mat& erosion_dst)
// {
//   int erosion_type;
//   if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
//   else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
//   else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
// 
//   cv::Mat element = cv::getStructuringElement( erosion_type,
//                                        cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
//                                        cv::Point( erosion_size, erosion_size ) );
// 
//   /// Apply the erosion operation
//   cv::erode( src, erosion_dst, element );
// //   imshow( "Erosion Demo", erosion_dst );
// }
// 
// /** @function Dilation */
// void Dilation( int dilation_elem, int dilation_size, cv::Mat const& src, cv::Mat& dilation_dst)
// {
//   int dilation_type;
//   if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
//   else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
//   else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
// 
//   cv::Mat element = cv::getStructuringElement( dilation_type,
//                                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//                                        cv::Point( dilation_size, dilation_size ) );
//   /// Apply the dilation operation
//   cv::dilate( src, dilation_dst, element );
// //   cv::imshow( "Dilation Demo", dilation_dst );
// }
// 
// void channel_processing(cv::Mat& channel)
// {
//      cv::adaptiveThreshold(channel, channel, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 55, 7);
//     // mop up the dirt
//      
//     Dilation(0, 5, channel, channel);
//     Erosion(0, 5, channel, channel);
// }
// 
// double inter_centre_distance(int x1, int y1, int x2, int y2)
// {
//     return sqrt((x1-x2)^2 + (y1-y2)^2);
// }
// 
// bool colliding_circles(vector<cv::Vec3f> const& circles)
// {
//     for (auto i = 0; i < circles.size(); ++i)
//     {
//         for (auto j = i+1; j < circles.size(); ++j)
// 	{
// 	    int x1 = cvRound(circles[i][0]);
// 	    int y1 = cvRound(circles[i][1]);
// 	    int radius1 = cvRound(circles[i][2]);
// 	    
// 	    int x2 = cvRound(circles[j][0]);
// 	    int y2 = cvRound(circles[j][1]);
// 	    int radius2 = cvRound(circles[j][2]);
// 
//             // collision or containment:
//             if (inter_centre_distance(x1,y1,x2,y2) < radius1 + radius2)
//                 return true;
// 	}
//     }
//     return false;
// }
// 
// void find_circles(cv::Mat const& processed, vector<cv::Vec3f> & storage, int &low)
// {
//   if (low > 105)
//     return;
//   
//   cv::HoughCircles(processed, storage, CV_HOUGH_GRADIENT, 2, 32.0, 30, low);//, 0, 100) great to add circle constraint sizes.
//   
//   if (storage.size() == 0)
//   {
//     low += 1;
//     find_circles(processed, storage, low);
//   }
//   
//   if (colliding_circles(storage))
//   {
//     low += 1;
//     find_circles(processed, storage, low);
//   }
//   
//   return; 
// }

// /////////////////////////////////////////////
// void detectCircle(
//   cv::Mat const& orig, cv::Mat & output, 
//   int dp_, int min_dist_, int cannyEdge_, int centerDetect_, int minrad_, int maxrad_)
// {
//   // create tmp images
//   cv::Mat l_empty_copy;
//   
//   std::vector<cv::Mat> l_channels;
//   l_channels.push_back(l_empty_copy);
//   l_channels.push_back(l_empty_copy);
//   l_channels.push_back(l_empty_copy);
//   
//   cv::split(orig, l_channels);
//   
//   for(size_t i = 0; i < l_channels.size(); ++i)
//   {
//     channel_processing(l_channels[i]);
//   }
//   
//   cv::merge(&l_channels[0], l_channels.size(), output);
//     
//   
//   cv::Canny(output, output, 5, 70, 3);
//   
//   int low = 100;
//   vector<cv::Vec3f> circles;
//   find_circles(output, circles, low);
//   
// }
// 
// /////////////////////////////////////////////
// void detectCircle(
//   cv::Mat const& input, cv::Mat & output, std::vector<ColorName> & colors,
//   int dp_, int min_dist_, int cannyEdge_, int centerDetect_, int minrad_, int maxrad_)
// {
//   cv::medianBlur(input, input, 3);
//   
//   // Convert input image to HSV
//   cv::Mat hsv_image;
//   cv::cvtColor(input, hsv_image, cv::COLOR_BGR2HSV);
// 
//   // Threshold the HSV image, keep only the red pixels
//   
//   cv::Mat hue_range = hsv_image.clone();
//      
//   cv::Mat l_gray = hue_range.clone();
//   
//   cv::GaussianBlur( l_gray, l_gray, cv::Size(9, 9), 2, 2 );
//   
//   cv::Mat l_canny = l_gray.clone(); 
//   cv::Canny(l_gray, l_canny, 200, 20);
// 
//   vector<cv::Vec3f> circles;
// 
//   /// Apply the Hough Transform to find the circles
//   cv::HoughCircles( l_canny, circles, CV_HOUGH_GRADIENT, dp_>0?dp_:1, min_dist_>0?min_dist_:1, cannyEdge_>0?cannyEdge_:1, centerDetect_>0?centerDetect_:1, minrad_>0?minrad_:0, maxrad_>0?maxrad_:0);
// 
//   output = input.clone();
//   /// Draw the circles detected
//   for( size_t i = 0; i < circles.size(); i++ )
//   {
//       cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//       int radius = cvRound(circles[i][2]);
//       // circle center
//       cv::circle( output, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//       // circle outline
//       cv::circle( output, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//    }
// }
// 
// 
// /////////////////////////////////////////////
// void Collection::ImageFromKinect(const sensor_msgs::ImageConstPtr& msg)
// {
//   Lock l_lck(m_mutex);
//   
//   cv_bridge::CvImageConstPtr cv_ptr;
//   try
//   {
//     if (sensor_msgs::image_encodings::isColor(msg->encoding))
// 	cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
//     else
// 	cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }
//   
//   cv::Mat l_subtract = cv_ptr->image.clone();
//   if (m_foregroundFLAG)
//     cv::subtract(cv_ptr->image, m_foreground, l_subtract);
//   
//   cv::threshold(l_subtract,l_subtract,m_thr,m_maxval,cv::THRESH_BINARY);
//   
//   // Update GUI Window
//   if (m_foregroundFLAG)
//   {
//     cv::imshow("Subtraction Image", l_subtract);
//     cv::waitKey(3);
//   }
// 
//   cv::Mat output;
//   std::vector<ColorName> l_colors;
//   l_colors.push_back(ColorName::red);
//   l_colors.push_back(ColorName::blue);
//   detectCircle(l_subtract, output, l_colors,
// 	       m_dp, m_min_dist, m_cannyEdge, m_centerDetect, m_minrad, m_maxrad);
// //   detectCircle(l_subtract, output, 
// // 	       m_dp, m_min_dist, m_cannyEdge, m_centerDetect, m_minrad, m_maxrad);
//   
//   // Update GUI Window
//   cv::imshow(OPENCV_WINDOW, output);
//   cv::waitKey(3);
//   
//   // Output modified video stream
//   m_image_pub.publish(cv_ptr->toImageMsg());
//   
//   m_data.insert( std::make_pair(Color(255,0,0), AgentSensor(5,5,15)) );
//   m_available=true;
// }
// 
// /**
//  * Helper function to find a cosine of angle between vectors
//  * from pt0->pt1 and pt0->pt2
//  */
// static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
// {
// 	double dx1 = pt1.x - pt0.x;
// 	double dy1 = pt1.y - pt0.y;
// 	double dx2 = pt2.x - pt0.x;
// 	double dy2 = pt2.y - pt0.y;
// 	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
// }
// 
// /**
//  * Helper function to display text in the center of a contour
//  */
// void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
// {
// 	int fontface = cv::FONT_HERSHEY_SIMPLEX;
// 	double scale = 0.4;
// 	int thickness = 1;
// 	int baseline = 0;
// 
// 	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
// 	cv::Rect r = cv::boundingRect(contour);
// 
// 	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
// 	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
// 	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
// }
// 
// /////////////////////////////////////////////
// void ImageFromKinectProcess(cv::Mat const& input, cv::Mat & output)
// {
//   // Convert to grayscale
// 
//   cv::Mat gray;
//   cv::cvtColor(input, gray, CV_BGR2GRAY);
//   
//   // Convert to binary image using Canny
//   cv::Mat bw;
//   cv::Canny(gray, bw, 0, 50, 5);
//   
//   // Find contours
//   std::vector<std::vector<cv::Point> > contours;
//   cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//   
//   // The array for storing the approximation curve
//   std::vector<cv::Point> approx;
// 
//   // We'll put the labels in this destination image
//   output = input.clone();
// 
// for (int i = 0; i < contours.size(); i++)
// {
//     // Approximate contour with accuracy proportional
//     // to the contour perimeter
//     cv::approxPolyDP(
//         cv::Mat(contours[i]), 
//         approx, 
//         cv::arcLength(cv::Mat(contours[i]), true) * 0.02, 
//         true
//     );
// 
//     // Skip small or non-convex objects 
//     if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
//         continue;
//     
//     if (approx.size() == 3)
//     {
//         setLabel(output, "TRI", contours[i]);    // Triangles
//     }
//     else if (approx.size() >= 4 && approx.size() <= 6)
//     {
//         // Number of vertices of polygonal curve
//         int vtc = approx.size();
// 
//         // Get the degree (in cosines) of all corners
//         std::vector<double> cos;
//         for (int j = 2; j < vtc+1; j++)
//             cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));
// 
//         // Sort ascending the corner degree values
//         std::sort(cos.begin(), cos.end());
// 
//         // Get the lowest and the highest degree
//         double mincos = cos.front();
//         double maxcos = cos.back();
// 
//         // Use the degrees obtained above and the number of vertices
//         // to determine the shape of the contour
//         if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
//         {
//             // Detect rectangle or square
//             cv::Rect r = cv::boundingRect(contours[i]);
//             double ratio = std::abs(1 - (double)r.width / r.height);
// 
//             setLabel(output, ratio <= 0.02 ? "SQU" : "RECT", contours[i]);
//         }
//         else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
//             setLabel(output, "PENTA", contours[i]);
//         else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
//             setLabel(output, "HEXA", contours[i]);
//     }
//     else
// 	{
// 	    // Detect and label circles
// 	    double area = cv::contourArea(contours[i]);
// 	    cv::Rect r = cv::boundingRect(contours[i]);
// 	    int radius = r.width / 2;
// 
// 	    if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
// 		std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
// 	    {
// 		setLabel(output, "CIR", contours[i]);
// 		
// 		vector<cv::Point> tmp = approx;
// 		const cv::Point* elementPoints[1] = { &tmp[0] };
// 		int numberOfPoints = (int)tmp.size();
// 		  
// 		cv::polylines( output, elementPoints, &numberOfPoints, 1, true, cv::Scalar(255,0,0) );
// 	    }
// 	}
//     } // end of for() loop
//     
//   
//   cv::Mat median_image;
//   cv::medianBlur(input, median_image, 3);
//  
//   // Convert input image to HSV
//   cv::Mat hsv_image;
//   cv::cvtColor(median_image, hsv_image, cv::COLOR_BGR2HSV);
//   
//   // Threshold the HSV image, keep only the red pixels
//   cv::Mat lower_red_hue_range;
//   cv::Mat upper_red_hue_range;
//   cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
//   cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
//   
// //   // Draw an example circle on the video stream
// //   if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
// //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
// }
// 
// /////////////////////////////////////////////
// nostop_kinect_sensor::SensorData Collection::getMsgs()
// {
//   Lock l_lck(m_mutex);
//   std::vector<nostop_kinect_sensor::AgentSensorData> l_result;
//   if(m_available)
//   {
//     for(auto it = m_data.begin(); it!= m_data.end(); ++it)
//     {
//       nostop_kinect_sensor::AgentSensorData l_elem;
// 	Color l_color = (*it).first;
// 	l_elem.r = l_color.r;
// 	l_elem.g = l_color.g;
// 	l_elem.b = l_color.b;
// 	AgentSensor l_agSens = (*it).second;
// 	l_elem.x = l_agSens.x;
// 	l_elem.y = l_agSens.y;
// 	l_elem.heading = l_agSens.heading;
// 	
// 	l_result.push_back(l_elem);
//     }
//     m_available=false;
//   }
//   nostop_kinect_sensor::SensorData l_msgs;
//   l_msgs.data = l_result;
//   return l_msgs;
// }
// 
