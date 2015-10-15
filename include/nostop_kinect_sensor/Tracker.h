////////////////////////////////////////////////////////////
//	Tracker.h
//	Created on:	15-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef TRACKER_H
#define TRACKER_H
#pragma once

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		
		class Tracker
		{
			int m_test;

		protected:
			
		public:
			Tracker();
			void test();
			int passaggio();
			~Tracker();
		};

	}
}


#endif // TRACKER_H