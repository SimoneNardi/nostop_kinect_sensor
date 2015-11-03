#include "Collector.h"
#include "Collection.h"
#include "std_msgs/String.h"
#include "some_struct.h"
#include <Robot_manager.h>

// TEST
#include "nostop_agent/Id_robot.h"
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
Collector::Collector():
m_number_kinect(0)
, m_number_another(0)
{
  ROS_INFO("Sensor: Collector init.");
  
 m_camera_in = m_node.subscribe<nostop_agent::Id_robot>("/test_camera", 1000, &Collector::new_camera,this);
 m_manager = std::make_shared<Robot_manager>();
}

/////////////////////////////////////////////
Collector::~Collector()
{}


void Collector::new_camera(const nostop_agent::Id_robot::ConstPtr& msg)// DA CAMBIARE
{	
    std::string name_number;
    name_number.assign( msg->name);
    if(msg->name == "kinect")
    {
      m_number_kinect = m_number_kinect+1;
      name_number = name_number+"_"+std::to_string(m_number_kinect);
    }else{
      m_number_another = m_number_another+1;
      name_number = name_number+"_"+std::to_string(m_number_another);
    }
    m_camera_array.push_back( std::make_shared<Collection>(name_number) );
    ROS_INFO("New CAMERA!");
}

void Collector::pack_passage()
{
  
  // CONVERSIONE IN SR W (in Collection)
  
  // TUTTI INSIEME
    for(size_t i = 0; i<m_camera_array.size();i++)
    {
        std::vector<ball_position> l_blue,l_green,l_red,l_yellow;
	l_blue = m_camera_array[i]->get_blue_array();
	l_green = m_camera_array[i]->get_green_array();
	l_red = m_camera_array[i]->get_red_array();
	l_yellow = m_camera_array[i]->get_yellow_array();
      for(size_t j=0;j<l_blue.size();j++)
      {
	m_blue_ball_W.push_back(l_blue[j]);
      }
      for(size_t j=0;j<l_green.size();j++)
      {
	m_green_ball_W.push_back(l_green[j]);
      }
      for(size_t j=0;j<l_red.size();j++)
      {
	m_red_ball_W.push_back(l_red[j]);
      }
      for(size_t j=0;j<l_yellow.size();j++)
      {
	m_yellow_ball_W.push_back(l_yellow[j]);
      }
    }
    
    
    // TO ROBOT MANAGER
    m_manager->array_assignment(m_blue_ball_W,m_green_ball_W,m_red_ball_W,m_yellow_ball_W);
    
    // CLEARING
    m_blue_ball_W.clear();
    m_green_ball_W.clear();
    m_red_ball_W.clear();
    m_yellow_ball_W.clear();
  }



//void Collector::run()
//{
// 	ros::Rate loop_rate(5);
// 
	
// 	ROS_INFO("Sensor Collector is running.");
// 
// 	int count = 0;
// 	while (ros::ok())
// 	{
// 	  m_blue_ball_W.clear();
// 	  m_green_ball_W.clear();
// 	  m_red_ball_W.clear();
// 	  m_yellow_ball_W.clear();
// 	  package();
// 	}
//   
//}




