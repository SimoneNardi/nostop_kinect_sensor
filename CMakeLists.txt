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
  rospy
  std_msgs 
  dynamic_reconfigure
  message_generation 
  rosbag
  base_math
  image_transport
  cv_bridge
  nostop_agent
  nostop_learning
  tf
  )

generate_dynamic_reconfigure_options(
   cfg/calibration_values.cfg
   )
   
find_package( OpenCV REQUIRED )





################################################
## Declare ROS messages, services and actions ##
################################################


add_service_files(
  FILES
  Camera_data_srv.srv
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
  Camera_data_msg.msg
)

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
  DEPENDS roscpp rospy base_math nostop_agent image_transport cv_bridge nostop_learning dynamic_reconfigure Geographic
)



###########
## Build ##
###########



    
include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS}
${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.
include_directories(${OpenCV_INCLUDE_DIRS})

set(sensor_SOURCES
        src/Camera.cpp
        src/Camera_manager.cpp
        src/acquisition_node.cpp
        src/Ball_tracker.cpp
        src/Robot_manager.cpp
        src/Robot.cpp

)

set(calibration_SOURCES
        src/calibration.cpp

)

set(cam_view_angle_SOURCES
	src/cam_view_angle.cpp
)


add_executable(acquisition_node ${sensor_SOURCES})
add_dependencies(acquisition_node nostop_kinect_sensor_gencpp)
target_link_libraries(acquisition_node ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon nostop_agent nostop_learning Geographic )

add_executable(calibration ${calibration_SOURCES})
add_dependencies(calibration nostop_kinect_sensor_gencpp  nostop_kinect_sensor_gencfg )
target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon nostop_agent nostop_learning )

add_executable(cam_view_angle ${cam_view_angle_SOURCES})
add_dependencies(cam_view_angle nostop_kinect_sensor_gencpp)
target_link_libraries(cam_view_angle ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon nostop_agent nostop_learning )


add_executable(test_Hough src/test.cpp)
add_dependencies(test_Hough nostop_kinect_sensor_gencpp)
target_link_libraries(test_Hough ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon nostop_agent nostop_learning )


#############
## Install ##
#############


 install(TARGETS acquisition_node
 EXPORT nostop_kinect_sensor
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(TARGETS cam_view_angle
 EXPORT nostop_kinect_sensor
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
