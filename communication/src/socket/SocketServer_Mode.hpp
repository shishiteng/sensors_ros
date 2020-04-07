#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include "turtle_actionlib/Velocity.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"




using namespace std;
//using namespace boost;

class SocketServer_Mode
{   
 public:
   ros::Publisher socket_pub;
   ros::Subscriber socket_sub;

    SocketServer_Mode();
    int ServerInit();
    int Read_thread();
    int Send(unsigned char *send_buf,int len);
    void cmdVelocityCallback(const geometry_msgs::Twist &twist);
    //void sig_handle(int sig);
    //bool comm_flag;
 private:
    int serverSock;
    int clientSock;
    //sig_atomic_t sig_flag;
    
       
};


//


