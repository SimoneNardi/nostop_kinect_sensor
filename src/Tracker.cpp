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

#include "Tracker.h"


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace cv;


Tracker::Tracker() : m_test(10) {}

Tracker::~Tracker() {}

void Tracker::test()
{ int num;
  num = m_test;
  ROS_INFO("%d",num);
}

int Tracker::passaggio()
{ 
  return m_test;
}