cmake_minimum_required(VERSION 2.8.3)
project(extended_kalman_filter)


set(VSE_FUSION_NODE_EKF_SIM vse_ekf_sim)
set(VSE_FUSION_NODE_EKF_CLASS_SIM vse_ekf_class)
set(VSE_TF_NODE_EKF_SIM vse_tf_frame_publisher_ekf_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  rosbag
  geometry_msgs
  cv_bridge
  image_transport
  rospy
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost)#REQUIRED COMPONENTS system)
find_package(OpenCV)

 add_message_files(
   FILES
   TempGpsData.msg 
   TempImuData.msg
   VSEFusionData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES extended_kalman_filter
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs cv_bridge image_transport message_runtime 
  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(OpenCV)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   include/ekf/
   ${OpenCV_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -g ${CMAKE_CXX_FLAGS}")

############
add_executable(${VSE_FUSION_NODE_EKF_SIM} src/ekf/${VSE_FUSION_NODE_EKF_SIM}.cpp src/ekf/${VSE_FUSION_NODE_EKF_CLASS_SIM}.cpp)
target_link_libraries(${VSE_FUSION_NODE_EKF_SIM} ${catkin_LIBRARIES})
add_dependencies(${VSE_FUSION_NODE_EKF_SIM} extended_kalman_filter_generate_messages_cpp)

add_executable(${VSE_TF_NODE_EKF_SIM} src/ekf/${VSE_TF_NODE_EKF_SIM}.cpp)
target_link_libraries(${VSE_TF_NODE_EKF_SIM} ${catkin_LIBRARIES})
add_dependencies(${VSE_TF_NODE_EKF_SIM} extended_kalman_filter_generate_messages_cpp)

install(FILES
   # myfile1
   cfg/ekf/ivs_vse_param.yaml
   cfg/ekf/ivs_vse_param_ekf.yaml
   cfg/ekf/odom_pose.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
