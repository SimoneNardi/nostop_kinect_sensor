////////////////////////////////////////////////////////////
//	Ball_tracker.h
//	Created on:	15-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef BALLTRACKER_H
#define BALLTRACKER_H
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "some_struct.h"
#include <memory>
#include "Camera.h"

namespace Robotics 
{
	namespace GameTheory
	{
		
		class Ball_tracker
		{
			int m_stateSize = 6, m_measSize = 4, m_contrSize = 0;
			unsigned int type = CV_32F;
			cv::KalmanFilter m_kf;
			cv::Mat m_state;
			cv::Mat m_meas;
			double m_ticks;
			ros::NodeHandle tracker;

		public:
			Ball_tracker();
			void matrixSettings(cv::KalmanFilter m_kf);
			cv::Rect kalman_update(ball_position& position);

			
			~Ball_tracker();
		};

	}
}


#endif // BALLTRACKER_H