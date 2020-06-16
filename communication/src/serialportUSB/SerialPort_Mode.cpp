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
#include <queue>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "SerialPort_Mode.hpp"

//宏定义
#define FALSE -1
#define TRUE 0
#define CONNECT_NUM 5
#define MAX_NUM 80
#define UART_DEVICE0 "/dev/ttyUSB0"

using namespace std;

static struct termios old_termios;

geometry_msgs::Pose EKF_pos;

bool isInitIMU = false;
double accx, accy, accz, avx, avy, avz;

SerialPort_Mode::SerialPort_Mode(std::string &port, unsigned int rate)
{
  Open(port);
  ttyport = port;
  bandrate = rate;
  Init(SerialPort_Mode::fd, rate);
}

int SerialPort_Mode::Open(std::string devname)
{
  int fd = open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    perror(UART_DEVICE0);
    return -1;
  }
  tcgetattr(fd, &old_termios); //save old termios
  SerialPort_Mode::fd = fd;
  return 0;
}

int SerialPort_Mode::Init(int fd, int baudrate)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(SerialPort_Mode::fd, &tty) != 0)
  {
    printf("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed(&tty, baudrate);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_iflag &= ~IGNBRK;                     // disable break processing
  tty.c_lflag = 0;                            // no signaling chars, no echo,
  tty.c_oflag = 0;                            // no remapping, no delays
  tty.c_cc[VMIN] = 1;                         // read block
  tty.c_cc[VTIME] = 0;                        // 0.5 seconds read timeout
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls,
  tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
  tty.c_cflag |= 0;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  if (tcsetattr(SerialPort_Mode::fd, TCSANOW, &tty) != 0)
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

  unsigned char recv_temp[1024];
  unsigned char *rcv_buf;
  string recv_data;
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
      memset(recv_temp, 0, sizeof(recv_temp));
      //len = read(fd, rcv_buf, 1024);
      // ROS_INFO("len=%d", len);
      int temp_len = read(fd, recv_temp, 1024);
      for (int i = 0; i < temp_len; i++)
      {
        recv_data += recv_temp[i];
      }

      len = recv_data.length();
      rcv_buf = (unsigned char *)recv_data.data();
      if (len >= 127)
      {
        for (int i = 0; i <= len - 3; i++)
        {
          if (rcv_buf[i] == 0x59 && rcv_buf[i + 1] == 0x49 && rcv_buf[i + 2] == 0x53)
          {
            if ((i + 127) > len) //
            {
              recv_data.erase(0, i);
              break;
            }

            if (!isInitIMU)
            {
              if (rcv_buf[i + 6] == 0x10)
              {
                accx = (rcv_buf[i + 8] << 0 | rcv_buf[i + 9] << 8 | rcv_buf[i + 10] << 16 | rcv_buf[i + 11] << 24) * 0.000001;
                accy = (rcv_buf[i + 12] << 0 | rcv_buf[i + 13] << 8 | rcv_buf[i + 14] << 16 | rcv_buf[i + 15] << 24) * 0.000001;
                accz = (rcv_buf[i + 16] << 0 | rcv_buf[i + 17] << 8 | rcv_buf[i + 18] << 16 | rcv_buf[i + 19] << 24) * 0.000001;
                //accz = (rcv_buf[i+30]<<0|rcv_buf[i+31]<<8|rcv_buf[i+32]<<16|rcv_buf[i+33]<<24)*0.000001;
                ROS_INFO("acx=%f,acy=%f,acz=%f", accx, accy, accz);
              }

              if (rcv_buf[i + 48] == 0x20)
              {
                avx = (rcv_buf[i + 50] << 0 | rcv_buf[i + 51] << 8 | rcv_buf[i + 52] << 16 | rcv_buf[i + 53] << 24) * 0.000001 * 3.141592653 / 180.0;
                avy = (rcv_buf[i + 54] << 0 | rcv_buf[i + 55] << 8 | rcv_buf[i + 56] << 16 | rcv_buf[i + 57] << 24) * 0.000001 * 3.141592653 / 180.0;
                avz = (rcv_buf[i + 58] << 0 | rcv_buf[i + 59] << 8 | rcv_buf[i + 60] << 16 | rcv_buf[i + 61] << 24) * 0.000001 * 3.141592653 / 180.0;
                ROS_INFO("avx=%f,avy=%f,avz=%f", avx, avy, avz);
              }
              isInitIMU = true;
              recv_data.erase(0, i + 127);
              continue;
            }

            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";

            if (rcv_buf[i + 6] == 0x10)
            {
              imu_data.linear_acceleration.x = (rcv_buf[i + 8] << 0 | rcv_buf[i + 9] << 8 | rcv_buf[i + 10] << 16 | rcv_buf[i + 11] << 24) * 0.000001;
              imu_data.linear_acceleration.y = (rcv_buf[i + 12] << 0 | rcv_buf[i + 13] << 8 | rcv_buf[i + 14] << 16 | rcv_buf[i + 15] << 24) * 0.000001;
              imu_data.linear_acceleration.z = (rcv_buf[i + 16] << 0 | rcv_buf[i + 17] << 8 | rcv_buf[i + 18] << 16 | rcv_buf[i + 19] << 24) * 0.000001;
              // ROS_INFO("acc %f %f %f", imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
            }

            if (rcv_buf[i + 48] == 0x20)
            {
              imu_data.angular_velocity.x = (rcv_buf[i + 50] << 0 | rcv_buf[i + 51] << 8 | rcv_buf[i + 52] << 16 | rcv_buf[i + 53] << 24) * 0.000001 * 3.141592653 / 180.0 - avx;
              imu_data.angular_velocity.y = (rcv_buf[i + 54] << 0 | rcv_buf[i + 55] << 8 | rcv_buf[i + 56] << 16 | rcv_buf[i + 57] << 24) * 0.000001 * 3.141592653 / 180.0 - avy;
              imu_data.angular_velocity.z = (rcv_buf[i + 58] << 0 | rcv_buf[i + 59] << 8 | rcv_buf[i + 60] << 16 | rcv_buf[i + 61] << 24) * 0.000001 * 3.141592653 / 180.0 - avz;
              // ROS_INFO("gyr %f %f %f", imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
            }

            if (rcv_buf[i + 76] == 0x40)
            {
              float pitch = (rcv_buf[i + 78] << 0 | rcv_buf[i + 79] << 8 | rcv_buf[i + 80] << 16 | rcv_buf[i + 81] << 24) * 0.000001;
              float roll = (rcv_buf[i + 82] << 0 | rcv_buf[i + 83] << 8 | rcv_buf[i + 84] << 16 | rcv_buf[i + 85] << 24) * 0.000001;
              float yaw = (rcv_buf[i + 86] << 0 | rcv_buf[i + 87] << 8 | rcv_buf[i + 88] << 16 | rcv_buf[i + 89] << 24) * 0.000001;

              // 模组输出的四元数和欧拉角有问题,会跳变
              float s = 3.141592653 / 180.f;
              imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * s, pitch * s, yaw * s);
              ROS_INFO("rpy %f %f %f", roll, pitch, yaw);

              double as = 0.000244141 * 9.8;
              double gs = 0.06103515625 * 3.141592653 / 180.0;

              // 标定后减去bias
              // imu_data.angular_velocity.x -= -0.283143 * gs - 0.0025;
              // imu_data.angular_velocity.y -= 0.65107 * gs;
              // imu_data.angular_velocity.z -= 0.00359928 * gs;
              // imu_data.linear_acceleration.x -= 9.72516 * as;
              // imu_data.linear_acceleration.y -= -2.64779 * as;
              // imu_data.linear_acceleration.z -= -12.1863 * as;

              double norm2 = pow(imu_data.linear_acceleration.x, 2) +
                             pow(imu_data.linear_acceleration.y, 2) +
                             pow(imu_data.linear_acceleration.z, 2);
              fprintf(stderr, "%f %4d %4d %4d %4d %4d %4d %.3f\n", imu_data.header.stamp.toSec(),
                      (int)(imu_data.angular_velocity.x / gs),
                      (int)(imu_data.angular_velocity.y / gs),
                      (int)(imu_data.angular_velocity.z / gs),
                      (int)(imu_data.linear_acceleration.x / as),
                      (int)(imu_data.linear_acceleration.y / as),
                      (int)(imu_data.linear_acceleration.z / as),
                      sqrt(norm2));
            }

            if (rcv_buf[i + 90] == 0x41)
            {
              //imu_data.orientation.w = (rcv_buf[i+92]<<0|rcv_buf[i+93]<<8|rcv_buf[i+94]<<16|rcv_buf[i+95]<<24)*0.000001;
              //imu_data.orientation.x = (rcv_buf[i+96]<<0|rcv_buf[i+97]<<8|rcv_buf[i+98]<<16|rcv_buf[i+99]<<24)*0.000001;
              //imu_data.orientation.y = (rcv_buf[i+100]<<0|rcv_buf[i+101]<<8|rcv_buf[i+102]<<16|rcv_buf[i+103]<<24)*0.000001;
              //imu_data.orientation.z = (rcv_buf[i+104]<<0|rcv_buf[i+105]<<8|rcv_buf[i+106]<<16|rcv_buf[i+107]<<24)*0.000001;
              //ROS_INFO("ox=%f,oy=%f,oz=%f,ow=%f",imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w);
            }
            recv_data.erase(0, i + 127);

            IMU_pub.publish(imu_data);

            geometry_msgs::PoseStamped pose_data;
            pose_data.header = imu_data.header;
            pose_data.pose.orientation = imu_data.orientation;
            pose_pub.publish(pose_data);
            break;
          }
        }
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
}

int SerialPort_Mode::Send(unsigned char *send_buf, int len)
{
  if (len = write(SerialPort_Mode::fd, send_buf, len))
  {
    return 0;
  }
  tcflush(fd, TCOFLUSH);
  return -1;
}

void SerialPort_Mode::cmdVelocityCallback(const geometry_msgs::Twist &twist)
{
  unsigned char sendbuf[10];
  short w = (short)(twist.angular.z * 10000);
  short v = (short)(twist.linear.x * 1000);
  if (w < 0)
    w += 255;
  sendbuf[0] = 0xAA;
  sendbuf[1] = 0x01;
  sendbuf[2] = 0x80;
  sendbuf[3] = 0x00;
  sendbuf[4] = 0x04;
  sendbuf[5] = (char)(w >> 8);
  sendbuf[6] = (char)(w >> 0);
  sendbuf[7] = (char)(v >> 8);
  sendbuf[8] = (char)(v >> 0);
  sendbuf[9] = sendbuf[5] ^ sendbuf[6] ^ sendbuf[7] ^ sendbuf[8];
  SerialPort_Mode::Send(sendbuf, 10);

  ROS_INFO(" cmdV=%.2f cmdW=%.3f", twist.linear.x, twist.angular.z);
}

int SerialPort_Mode::Close(int fd)
{
  tcsetattr(fd, TCSANOW, &old_termios); // restore
  close(fd);
  return 0;
}
