cmake_minimum_required(VERSION 2.8.3)
project(communication)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  message_generation
)


 add_message_files(
   DIRECTORY msg
   FILES
   vehicleData.msg
   vehicle_general_data.msg
   vehicle_sensor_data.msg
   cloudCmd.msg
   cloud_chassis_cmd.msg
   cloud_move_cmd.msg
   
   #Message2.msg
 )


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )



## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set (SOCKETSERVER
    src/socket/SocketServer_Mode.cpp
    src/socket/SocketServer_Node.cpp
)

add_executable(socket ${SOCKETSERVER})
target_link_libraries(socket ${catkin_LIBRARIES})



set (SERIALPORT
    src/serialport/SerialPort_Mode.cpp
    src/serialport/SerialPort_Node.cpp
)

add_executable(serialport ${SERIALPORT})
target_link_libraries(serialport ${catkin_LIBRARIES})

set (SERIALPORT
    src/serialportUSB/SerialPort_Mode.cpp
    src/serialportUSB/SerialPort_Node.cpp
)

add_executable(serialportUSB ${SERIALPORT})
target_link_libraries(serialportUSB ${catkin_LIBRARIES})
