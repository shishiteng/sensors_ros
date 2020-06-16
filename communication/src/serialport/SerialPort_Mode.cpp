#include <sys/types.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <semaphore.h>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "SerialPort_Mode.hpp"


//宏定义
#define FALSE  -1
#define TRUE   0
#define CONNECT_NUM 5
#define MAX_NUM 80
#define UART_DEVICE0     "/dev/ttyUSB0"




static struct termios old_termios;

geometry_msgs::Pose EKF_pos;

bool isUseEkFpos=false;

string recv_data;


SerialPort_Mode::SerialPort_Mode(std::string& port, unsigned int rate)
{	
   Open(port);
   ttyport = port;
   bandrate = rate;
   Init(SerialPort_Mode::fd,rate);
}

int SerialPort_Mode::Open(std::string devname)
{
  int fd = open(devname.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd<0)
  {
    perror(UART_DEVICE0);
    return -1;
  }
  tcgetattr(fd, &old_termios);//save old termios
  SerialPort_Mode::fd=fd;
  return 0;
}




int SerialPort_Mode::Init(int fd,int baudrate)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (SerialPort_Mode::fd, &tty) != 0)
  {
    printf("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, baudrate);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 1;            // read block
  tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= 0;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  if (tcsetattr (SerialPort_Mode::fd, TCSANOW, &tty) != 0) 
  {
     printf("error %d from tcsetattr", errno);
     return -1;
  }
  return 0;
}

void SerialPort_Mode::Read_thread(int fd)
{
	
    int len;
    fd_set fs_read;
    
    geometry_msgs::Quaternion odom_quat;

    unsigned char rcv_buf[1024];
    unsigned char *temp_buf;
    struct timeval time;
    time.tv_sec = 10;
    time.tv_usec = 0;
	
    geometry_msgs::Twist twist;
 
    ros::Rate loop_rate(10);

    while (ros::ok()) 
    {
        	
      FD_ZERO(&fs_read);
      FD_SET(fd, &fs_read);
      
      //使用select实现串口的多路通信
      if (select(fd + 1, &fs_read, NULL, NULL, &time))
      {
        memset(rcv_buf, 0, sizeof(rcv_buf));
        len = read(fd, rcv_buf, 1024);
        ROS_INFO("len=%d",len);
        if (len < 1)
	     continue;
        for(int i=0;i<len;i++)
        {
          recv_data += rcv_buf[i];
        }
        len = recv_data.length();
        
        if(len>=20)
        {
         temp_buf = (unsigned char*)recv_data.data();
         int i=0;
         while(i+20<=len)
         {
           if(temp_buf[i]==0xCC && temp_buf[i+4]==14)
           {
             twist.angular.z=(short(temp_buf[i+5]<<8|temp_buf[i+6]))/10000.0;	  
             twist.linear.x=(short(temp_buf[i+7]<<8|temp_buf[i+8]))/1000.0;
             //twist.linear.z=short(temp_buf[i+9]<<8|temp_buf[i+10])*3.14/180; //YAW from IMU
             ROS_INFO("back v=%.2f back w=%.2f back Yaw=%.2f",twist.linear.x,twist.angular.z,twist.linear.z);

             twist.angular.x=int(temp_buf[i+11]<<24|temp_buf[i+12]<<16|temp_buf[i+13]<<8|temp_buf[i+14]);
             twist.angular.y=int(temp_buf[i+15]<<24|temp_buf[i+16]<<16|temp_buf[i+17]<<8|temp_buf[i+18]);
             twist.linear.z= ros::Time::now().toSec();
             ROS_INFO("back Leftpos=%.2f Rightpos=%.2f",twist.angular.x,twist.angular.y);
             backVel_pub.publish(twist);
            
             i+=20;
           }
           else
           {
             i++;
           }
         }
         recv_data.erase(0,i);

        }   
      }
        
      loop_rate.sleep();
      ros::spinOnce();
    }
}


int SerialPort_Mode::Send(unsigned char *send_buf,int len)
{
    if(len = write(SerialPort_Mode::fd,send_buf,len))
    {
      return 0;
    }
    tcflush(fd,TCOFLUSH);
    return -1;
 }

 
void SerialPort_Mode::cmdVelocityCallback(const geometry_msgs::Twist &twist)
{
  unsigned char sendbuf[10];
  short w=(short)(twist.angular.z*10000);
  short v=(short)(twist.linear.x*1000);
  if(w<0)
   w+=255;
  sendbuf[0]=0xAA;
  sendbuf[1]=0x01;
  sendbuf[2]=0x80;
  sendbuf[3]=0x00;
  sendbuf[4]=0x04;
  sendbuf[5]=(char)(w>>8);
  sendbuf[6]=(char)(w>>0);
  sendbuf[7]=(char)(v>>8);
  sendbuf[8]=(char)(v>>0);
  sendbuf[9]=sendbuf[5]^sendbuf[6]^sendbuf[7]^sendbuf[8];
  SerialPort_Mode::Send(sendbuf,10);
  
  ROS_INFO("cmdV=%.2f cmdW=%.3f", twist.linear.x,twist.angular.z);
}
 
int SerialPort_Mode::Close(int fd){
	tcsetattr(fd, TCSANOW, &old_termios); // restore
	close(fd);
    return 0;
}


