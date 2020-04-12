cmake_minimum_required(VERSION 2.8.3)
project(simulation)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  roscpp
  tf
  std_msgs
  std_srvs
  dynamic_reconfigure
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
SET(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -g ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_service_files(
  FILES
  SetBias.srv
)


add_message_files(
FILES
gpsData.msg
imuData.msg
wheelData.msg
SimJoystickAndKeyboard.msg
SimulatorTwist.msg

)
 generate_messages(
   DEPENDENCIES
   roscpp
   std_msgs
   geometry_msgs
     sensor_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/SensorModel.cfg
  cfg/GNSS.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simulation
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime nav_msgs gazebo_ros
  DEPENDS roscpp
)

link_directories(/usr/lib/x86_64-linux-gnu) #for -lX11
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(include)


add_library(gps src/gazebo_ros_gps.cpp)
target_link_libraries(gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gps ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library(imu src/gazebo_ros_imu.cpp)
target_link_libraries(imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(imu ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library(car_controller_plugin_key_input SHARED src/car_controller_plugin_key_input.cpp)
target_link_libraries(car_controller_plugin_key_input ${GAZEBO_libraries} ${roscpp_LIBRARIES})
add_dependencies(car_controller_plugin_key_input ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(wheel_plugin SHARED src/wheel_plugin.cpp)
target_link_libraries(wheel_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
add_dependencies(wheel_plugin ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(keyboard_input src/keyboard_input.cpp)
target_link_libraries(keyboard_input ${catkin_LIBRARIES} -lX11)
add_dependencies(keyboard_input ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)



