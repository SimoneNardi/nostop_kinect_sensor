#include "Robot_manager.h"
#include "Robot.h"

#include "ros/ros.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;


Robot_manager::Robot_manager(): 
m_robot_count(0)
{
}

void Robot_manager::sub()
{
  m_robot_in = m_manager_node.subscribe("", 5, &Robot_manager::new_robot);
}

Robot_manager::~Robot_manager()	
{}

void Robot_manager::new_robot()// INPUT??
{
  // IF MSG OK
    m_robot_array[m_robot_count];// TO DO 
    m_robot_count++;
}
