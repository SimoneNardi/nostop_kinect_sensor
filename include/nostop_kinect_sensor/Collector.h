////////////////////////////////////////////////////////////
//	Collector.h
//	Created on:	07-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef COLLECTOR_H
#define COLLECTOR_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

#include "Threads.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class Collection;
		
		class Collector: public ThreadBase	  	
		{
		  mutable Mutex m_mutex;
		  
		  std::shared_ptr<Collection> m_sensor;
		  bool m_notify;
		  ros::NodeHandle m_node;
		  ros::Publisher m_pub;
		  
		protected:
			virtual void run();
		public:
			Collector();

			~Collector();
		};

	}
}


#endif // COLLECTOR_H