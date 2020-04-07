#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "turtle_actionlib/Velocity.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



using namespace std;
using namespace boost;


class SerialPort_Mode
{   

 public:
   int fd;
   ros::Publisher backVel_pub;
 
   ros::Publisher IMU_pub;
   ros::Publisher pose_pub;

   ros::Subscriber cmdVel_sub;


   SerialPort_Mode(std::string& port, unsigned int rate);
   int Send(unsigned char *send_buf,int len);
	
   int Open(std::string devname);
   int Init(int fd,int baudrate);
   void Read_thread(int fd);
   int Send(int fd, char *send_buf,int len);
   void cmdVelocityCallback(const geometry_msgs::Twist &twist);
   int Close(int fd);
	
	
private:
   std::string ttyport;
   int  bandrate;
};


//


