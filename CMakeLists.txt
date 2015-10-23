cmake_minimum_required(VERSION 2.8.3)
project(nostop_kinect_sensor)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  message_generation 
  rosbag
  base_math
  image_transport
  cv_bridge)

find_package( OpenCV REQUIRED )
# find_package( PCL 1.7.1 REQUIRED) # COMPONENT common io visualization)
# 
# include_directories(BEFORE ${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
#    AgentSensorData.msg
#    SensorData.msg
     Id_robot.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS base_math image_transport cv_bridge 
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.
include_directories(${OpenCV_INCLUDE_DIRS})

set(sensor_SOURCES
#         src/Sensor.cpp
#         src/SensorCollection.cpp
#         src/SensorCollector.cpp
        src/Collection.cpp
        src/Collector.cpp
        src/sensor_node.cpp
        src/Tracker.cpp
        src/Robot_manager.cpp
        src/Robot.cpp
)

# add_executable(Sensor ${sensor_SOURCES})
# target_link_libraries(Sensor ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon )#${PCL_LIBRARIES})
# add_dependencies(Sensor nostop_kinect_sensor_gencpp)


add_executable(sensor_node ${sensor_SOURCES})
target_link_libraries(sensor_node ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon )#${PCL_LIBRARIES})
add_dependencies(sensor_node nostop_kinect_sensor_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS Sensor
#  EXPORT nostop_kinect_sensor
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
 

 install(TARGETS sensor_node
 EXPORT nostop_kinect_sensor
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
