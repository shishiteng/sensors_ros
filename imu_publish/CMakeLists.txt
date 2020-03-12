cmake_minimum_required(VERSION 2.8.3)
project(imu_publish)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS saber_ros_inc
#  LIBRARIES imu_publish
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
 saber_ros_inc
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu main.cpp saber_ros_src/saber_config.cpp saber_ros_src/saber_protocol.cpp  saber_ros_src/saber_serial.cpp saber_ros_src/saber_tool.cpp config_tool/cJSON.c)
target_link_libraries(imu ${catkin_LIBRARIES})

#add_executable(receive src/receive.cpp)
#target_link_libraries(receive ${catkin_LIBRARIES})
