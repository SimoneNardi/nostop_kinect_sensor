////////////////////////////////////////////////////////////
//	Tracker.h
//	Created on:	15-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef TRACKER_H
#define TRACKER_H
#pragma once

#include <opencv2/core/core.hpp>

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		
		class Tracker
		{
			int m_stateSize = 6, m_measSize = 4, m_contrSize = 0;
			unsigned int type = CV_32F;
			cv::KalmanFilter m_kf;
			cv::Mat m_state;
			cv::Mat m_meas;
			cv::Mat m_drawCircle;
			double m_ticks;
			bool m_found;
			int m_notFoundCount = 0;
		  

		protected:
			
		public:
			Tracker();
			void matrixSettings(cv::KalmanFilter m_kf);
			void findCircles(cv::Mat thresholded_image, cv:: Mat drawCircle);
			~Tracker();
		};

	}
}


#endif // TRACKER_H