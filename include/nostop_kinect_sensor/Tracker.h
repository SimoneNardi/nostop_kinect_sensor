////////////////////////////////////////////////////////////
//	Tracker.h
//	Created on:	15-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef TRACKER_H
#define TRACKER_H
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "some_struct.h"
#include <memory>
#include "Collection.h"

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
			double m_ticks;
			ros::NodeHandle tracker;

		public:
			Tracker();
			void matrixSettings(cv::KalmanFilter m_kf);
			cv::Rect kalman_update(ball_position& position);

			
			~Tracker();
		};

	}
}


#endif // TRACKER_H