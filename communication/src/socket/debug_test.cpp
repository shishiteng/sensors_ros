/*************************************************************************
    > File Name: debug_test.cpp
    > Author: Louis.Qiu
    > Mail: louis.qiu@cloudminds.com
    > Created Time: Fri 22 Jun 2018 14:08:03 PM CST
 ************************************************************************/
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


#define PORT 3490 /* 客户机连接远程主机的端口 */

#define MAXDATASIZE 100 /* 每次可以接收的最大字节 */

int main(int argc, char *argv[])

{

int sockfd, numbytes;

char buf[MAXDATASIZE];

struct hostent *he;

struct sockaddr_in their_addr; /* connector's address information */

if (argc != 2) {

fprintf(stderr,"usage: client hostname\n");

exit(1);

}

if ((he=gethostbyname(argv[1])) == NULL) { /* get the host info */

herror("gethostbyname");

exit(1);

}

if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {

perror("socket");

exit(1);

}

their_addr.sin_family = AF_INET; /* host byte order */

their_addr.sin_port = htons(PORT); /* short, network byte order */

their_addr.sin_addr = *((struct in_addr *)he->h_addr);

bzero(&(their_addr.sin_zero),; /* zero the rest of the struct */

if(connect(sockfd,(struct sockaddr *)&their_addr,sizeof(struct sockaddr)) == -1) {

perror("connect");

exit(1);

}

if ((numbytes=recv(sockfd, buf, MAXDATASIZE, 0)) == -1) {

perror("recv");

exit(1);

}

buf[numbytes] = '\0';

printf("Received: %s",buf);

close(sockfd);

return 0;

}



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
