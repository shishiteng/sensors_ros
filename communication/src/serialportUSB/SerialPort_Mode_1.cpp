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
    struct timeval time;
    time.tv_sec = 10;
    time.tv_usec = 0;
	

    sensor_msgs::Imu imu_data;
    ros::Time current_time;

   ros::Rate loop_rate(100);
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
            if(rcv_buf[0]==0x59 &&rcv_buf[1]==0x49 && rcv_buf[2]==0x53 && len>=127)
            { 

                imu_data.header.stamp = ros::Time::now();
                imu_data.header.frame_id = "base_link";
  
                if(rcv_buf[6]==0x10)
                {
                  imu_data.linear_acceleration.x = (rcv_buf[8]<<0|rcv_buf[9]<<8|rcv_buf[10]<<16|rcv_buf[11]<<24)*0.000001;
                  imu_data.linear_acceleration.x = (rcv_buf[22]<<0|rcv_buf[23]<<8|rcv_buf[24]<<16|rcv_buf[25]<<24)*0.000001;
                  imu_data.linear_acceleration.y = (rcv_buf[12]<<0|rcv_buf[13]<<8|rcv_buf[14]<<16|rcv_buf[15]<<24)*0.000001;
                  imu_data.linear_acceleration.z = (rcv_buf[16]<<0|rcv_buf[17]<<8|rcv_buf[18]<<16|rcv_buf[19]<<24)*0.000001;
                  imu_data.linear_acceleration.z = (rcv_buf[30]<<0|rcv_buf[31]<<8|rcv_buf[32]<<16|rcv_buf[33]<<24)*0.000001;
                  ROS_INFO("acx=%f,acy=%f,acz=%f",imu_data.linear_acceleration.x,imu_data.linear_acceleration.y,imu_data.linear_acceleration.z);
                }
              
                if(rcv_buf[48]==0x20)
                {
                  imu_data.angular_velocity.x = (rcv_buf[50]<<0|rcv_buf[51]<<8|rcv_buf[52]<<16|rcv_buf[53]<<24)*0.000001;
                  imu_data.angular_velocity.y = (rcv_buf[54]<<0|rcv_buf[55]<<8|rcv_buf[56]<<16|rcv_buf[57]<<24)*0.000001;
                  imu_data.angular_velocity.z = (rcv_buf[58]<<0|rcv_buf[59]<<8|rcv_buf[60]<<16|rcv_buf[61]<<24)*0.000001;
                  ROS_INFO("avx=%f,avy=%f,avz=%f",imu_data.angular_velocity.x,imu_data.angular_velocity.y,imu_data.angular_velocity.z);
                }
        
                if(rcv_buf[90]==0x41)
                {                  
                  imu_data.orientation.x = (rcv_buf[92]<<0|rcv_buf[93]<<8|rcv_buf[94]<<16|rcv_buf[95]<<24)*0.000001;
                  imu_data.orientation.y = (rcv_buf[96]<<0|rcv_buf[97]<<8|rcv_buf[98]<<16|rcv_buf[99]<<24)*0.000001;
                  imu_data.orientation.z = (rcv_buf[100]<<0|rcv_buf[101]<<8|rcv_buf[102]<<16|rcv_buf[103]<<24)*0.000001;
                  imu_data.orientation.w = (rcv_buf[104]<<0|rcv_buf[105]<<8|rcv_buf[106]<<16|rcv_buf[107]<<24)*0.000001;
                  ROS_INFO("ox=%f,oy=%f,oz=%f",imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.x);
                }
            IMU_pub.publish(imu_data);
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


