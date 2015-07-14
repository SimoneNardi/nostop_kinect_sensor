////////////////////////////////////////////////////
//	SensorCollector.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef SENSOR_COLLECTOR_H
#define SENSOR_COLLECTOR_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

#include "Threads.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class SensorCollection;
	  
		class SensorCollector: public ThreadBase	  	
		{
		  mutable Mutex m_mutex;
		  
		  std::shared_ptr<SensorCollection> m_sensor;
		  
		  bool m_notify;
		  
		  ros::NodeHandle m_node;
		  ros::Publisher m_pub;
		  
		protected:
			virtual void run();
		public:
			SensorCollector();
			
			~SensorCollector();
		};

	}
}


#endif // SENSOR_COLLECTOR_H