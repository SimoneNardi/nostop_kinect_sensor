// Kalman Filter
    cv::setIdentity(m_kf.transitionMatrix);
    m_kf.measurementMatrix = cv::Mat::zeros(m_measSize,m_stateSize,m_type);
    m_kf.measurementMatrix.at<int64>(0) = 1.0f;
    m_kf.measurementMatrix.at<int64>(7) = 1.0f;
    m_kf.measurementMatrix.at<int64>(16) = 1.0f;
    m_kf.measurementMatrix.at<int64>(23) = 1.0f;
    
    double precTick = m_ticks;
    m_ticks = (double) cv::getTickCount();
 
     double dT = (m_ticks-precTick) / cv::getTickFrequency(); //seconds
 
      // Frame acquisition
     cv::Mat res;
     m_stream_video.copyTo(res);
     if (m_found)
          {
         // >>>> Matrix A
         m_kf.transitionMatrix.at<float>(2) = dT;
         m_kf.transitionMatrix.at<float>(9) = dT;
         // <<<< Matrix A
         m_state = m_kf.predict();  
	 cv::Rect predRect;          
	 predRect.width = m_state.at<float>(4);          
	 predRect.height = m_state.at<float>(5);          
	 predRect.x = m_state.at<float>(0) - predRect.width / 2;          
	 predRect.y = m_state.at<float>(1) - predRect.height / 2;            
	 cv::Point center;          
	 center.x = m_state.at<float>(0);          
	 center.y = m_state.at<float>(1);          
	 cv::circle(res, center, 2, CV_RGB(255,0,0), -1);            
	 cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);       
	
      }  
	
     

        // >>>>> Detection result
        for (size_t i = 0; i < balls.size(); i++)
        {
            cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
            cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);
        }
      // <<<<< Detection result         // >>>>> Kalman Update
      if (balls.size() == 0)
      {
         m_notFoundCount++;         
	 if( m_notFoundCount >= 100 )
         {
            m_found = false;
         }
//          else{
// 	    m_kf.statePost = m_state;
// 	 }
      }
      else
      {
	
         m_notFoundCount = 0;
 
         m_meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
         m_meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
         m_meas.at<float>(2) = (float)ballsBox[0].width;
         m_meas.at<float>(3) = (float)ballsBox[0].height;
	 

         if (!m_found) // First detection!
         {
	   ROS_INFO("FIRST DETECTION");
            // >>>> Initialization
            m_kf.errorCovPre.at<float>(0) = 1; // px
            m_kf.errorCovPre.at<float>(7) = 1; // px
            m_kf.errorCovPre.at<float>(14) = 1;
            m_kf.errorCovPre.at<float>(21) = 1;
            m_kf.errorCovPre.at<float>(28) = 1; // px
            m_kf.errorCovPre.at<float>(35) = 1; // px
 
            m_state.at<float>(0) = m_meas.at<float>(0);
            m_state.at<float>(1) = m_meas.at<float>(1);
            m_state.at<float>(2) = 0;
            m_state.at<float>(3) = 0;
            m_state.at<float>(4) = m_meas.at<float>(2);
            m_state.at<float>(5) = m_meas.at<float>(3);
            // <<<< Inizializzazione
 
            m_found = true;
         }
         else
	 {
            m_kf.correct(m_meas); // Kalman Correction
	 }
      }