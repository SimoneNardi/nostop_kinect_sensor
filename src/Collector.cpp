#include "Collector.h"
#include "Collection.h"


using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
Collector::Collector(): m_sensor(nullptr)
{
  ROS_INFO("Sensor: Collector init.");
    
  // Publish Sensor Information: 
  // COLLECTION
  m_sensor = std::make_shared<Collection>();

  m_sensor->subscribe();
  m_sensor->searchCircles();
}

/////////////////////////////////////////////
Collector::~Collector()
{}

/////////////////////////////////////////////
void Collector::run()
{
// 	ros::Rate loop_rate(5);
// 
// 	ROS_INFO("Sensor Collector is running.");
// 
// 	int count = 0;
// 	while (ros::ok())
// 	{
// 		nostop_kinect_sensor::SensorData l_srvData = m_sensor->getMsgs();
// 		m_pub.publish(l_srvData);
// 				
// 		ros::spinOnce();
// 
// 		loop_rate.sleep();
// 		++count;
// 
// 		ROS_DEBUG("Sensor Collector Run.");
// 	}
//   
}
