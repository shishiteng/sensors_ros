#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream> 

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp> //for ref_list_of  
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/progress.hpp>
#include <boost/program_options.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include "SocketServer_Mode.hpp"
#include <csignal>
#include <unistd.h>


SocketServer_Mode LServer;

void sig_stop(int sig)
{
  std::cout<<"exit socket thread!"<<std::endl;
  _exit(0);
}
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "SocketServer_Node");

	boost::shared_ptr< ros::NodeHandle> handel(new ros::NodeHandle); 
  signal(SIGINT,sig_stop);
  LServer.ServerInit();
	LServer.socket_pub = handel->advertise<geometry_msgs::Twist>("backVelocity", 5);
	
	LServer.socket_sub =handel->subscribe("cmdVelocity", 5, &SocketServer_Mode::cmdVelocityCallback,&LServer);
	     
	//create main thread
	boost::thread t1(boost::bind(&SocketServer_Mode::Read_thread,&LServer));  
	
	ros::spin();
	t1.join();

	return 0;
}





