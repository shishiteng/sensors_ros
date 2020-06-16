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



#include "SerialPort_Mode.hpp"



int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "uart_ctrlNode");

	std::string COM="/dev/ttyUSB0";
	int badrate=B115200;
	
	SerialPort_Mode LCCom(COM,badrate);
	
	boost::shared_ptr< ros::NodeHandle> handel(new ros::NodeHandle); 


	LCCom.backVel_pub = handel->advertise<geometry_msgs::Twist>("backVelocity", 5);

	
	LCCom.cmdVel_sub =handel->subscribe("cmdVelocity", 50, &SerialPort_Mode::cmdVelocityCallback,&LCCom);
	
	//crate main thread
	boost::thread t1(boost::bind(&SerialPort_Mode::Read_thread,&LCCom,LCCom.fd));  
	
	ros::spin();
	t1.join();

	return 0;


}





