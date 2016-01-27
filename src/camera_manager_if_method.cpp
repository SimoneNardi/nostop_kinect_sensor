// 		if(m_robot_initial_configuration[i].pose_setted == 0)
// 		{
// 			for(size_t j = 0;j<m_camera_on.size();j++)
// 			{
// 				m_camera_on[j].image = m_camera_array.at(j)->get_stream_video();
// 				std::string windows_name = m_robot_initial_configuration[i].name + " " + m_camera_on[j].camera_name  + " initial_pose";
// 				char* mouse_windows_name = new char[windows_name.size() + 1];
// 				std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
// 				mouse_windows_name[windows_name.size()] = '\0'; 
// 				cv::imshow(windows_name,m_camera_on[j].image );
// 				cvSetMouseCallback(mouse_windows_name,mouse_callback, &(m_robot_initial_configuration[i]) );
// 			}
// 		}else if ( m_robot_initial_configuration[i].pose_setted == 1 )
// 			{  
// 				cv::Point2f odom_SR;
// 				odom_SR.x = m_robot_initial_configuration[i].initial_pose_rect.x;
// 				odom_SR.y = m_robot_initial_configuration[i].initial_pose_rect.y;
// 				m_robot_initial_configuration[i].initial_point_odom_SR = odom_SR;
// 				for(size_t k = 0;k<m_camera_on.size();++k)
// 				{
// 					std::string windows_name = m_robot_initial_configuration[i].name + " " + m_camera_on[k].camera_name + " initial_pose";
// 					char* mouse_windows_name = new char[windows_name.size() + 1];
// 					std::copy(windows_name.begin(), windows_name.end(), mouse_windows_name);
// 					mouse_windows_name[windows_name.size()] = '\0'; 
// 					cvDestroyWindow(mouse_windows_name);
// 				}
// 				m_robot_initial_configuration[i].pose_setted = 2;
// 			  }else if(m_robot_initial_configuration[i].pose_setted ==2)
// 				{
// 					for(size_t h = 0;h<m_camera_on.size();++h)
// 					{
// 						m_camera_array.at(h)->robot_topic_pose_subscribe(m_robot_initial_configuration[i]);
// 					}
// 				}