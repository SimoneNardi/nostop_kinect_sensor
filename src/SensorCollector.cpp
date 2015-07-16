#include "SensorCollector.h"
#include "SensorCollection.h"

#include "nostop_kinect_sensor/SensorData.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
SensorCollector::SensorCollector()
: m_sensor(nullptr)
{
  //std::cout << "Sensor: Collector init!"<< std::endl << std::flush;
  ROS_INFO("Sensor: Collector init.");
    
  // Publish Sensor Information:
  m_pub = m_node.advertise<nostop_kinect_sensor::SensorData>("sensor", 5);
  m_sensor = std::make_shared<SensorCollection>();
  
  m_sensor->subscribe();
  
}

/////////////////////////////////////////////
SensorCollector::~SensorCollector()
{}

/////////////////////////////////////////////
void SensorCollector::run()
{
	ros::Rate loop_rate(5);

	//std::cout << "Sensor Collector is running!"<< std::endl << std::flush;
	ROS_INFO("Sensor Collector is running.");

	int count = 0;
	while (ros::ok())
	{
		nostop_kinect_sensor::SensorData l_srvData = m_sensor->getMsgs();
		m_pub.publish(l_srvData);
				
		ros::spinOnce();

		loop_rate.sleep();
		++count;

		ROS_DEBUG("Sensor Collector Run.");
	}
  
}
