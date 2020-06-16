#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h> /* memset */
#include <unistd.h> /* close */
#include <signal.h>


#include "SocketServer_Mode.hpp"

#define SRVPORT 8088
#define CONNECT_NUM 5
#define MAX_NUM 80

using namespace std;

int serverSock=-1,clientSock=-1;
struct sockaddr_in serverAddr;


SocketServer_Mode::SocketServer_Mode(){
	
}


int SocketServer_Mode::ServerInit()
{
    serverSock=socket(AF_INET,SOCK_STREAM,0);/**/
    if(serverSock<0)    { 
      printf("socket creation failed\n");
      return -1;
    }
    /**qsl*/
    int on=1;
    if(setsockopt(serverSock,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on)) == -1)
    { printf("setsockopt failed %s\n",strerror((errno)));
      return -1;
    }
    /**qsl*/
    printf("socket create successfully.\n");

    memset(&serverAddr,0,sizeof(serverAddr));
    serverAddr.sin_family=AF_INET;
    serverAddr.sin_port=htons((u_short) SRVPORT);
    serverAddr.sin_addr.s_addr=htons(INADDR_ANY);

   if(bind(serverSock,(struct sockaddr *)&serverAddr,sizeof(struct sockaddr))<0)
   {
      printf("Bind error.\n");
      return -1;
    }
    printf("Bind successful.\n");

    if(listen(serverSock,10)==-1){
        printf("Listen error!\n");
    }
    printf("Start to listen!\n");
	return 0;
}

int SocketServer_Mode::Read_thread()
{
    unsigned char rcv_buf[MAX_NUM]={0};
    socklen_t sin_size=sizeof(struct sockaddr_in);

    while(ros::ok())
    {      
      clientSock=accept(serverSock,(struct sockaddr *)&serverAddr,&sin_size);
      printf("接受到一个连接：%d \r\n", clientSock); 
     
      ros::Rate loop_rate(10);
      while(1)
      {
        int len	=read(clientSock,rcv_buf,MAX_NUM);
        if(len<1)
        {
           printf("read error.\n");
           break;
        }else{
         //ROS_INFO("%d %d %d %d %d %d %d %d %d",rcv_Buf[0],rcv_Buf[1],rcv_Buf[2],rcv_Buf[3],rcv_Buf[4],rcv_Buf[5],rcv_Buf[6],rcv_Buf[7],rcv_Buf[8]);
         
        }
        geometry_msgs::Twist twist;
        if(rcv_buf[0]==0xCC && len>=20)
        {
            twist.angular.z=(short(rcv_buf[5]<<8|rcv_buf[6]))/10000.0;
            twist.linear.x=(short(rcv_buf[7]<<8|rcv_buf[8]))/1000.0;
            twist.linear.z=short(rcv_buf[9]<<8|rcv_buf[10])*3.14/180; //YAW from IMU
            ROS_INFO("back v=%.2f back w=%.2f back Yaw=%.2f",twist.linear.x,twist.angular.z,twist.linear.z);

            twist.angular.x=int(rcv_buf[11]<<24|rcv_buf[12]<<16|rcv_buf[13]<<8|rcv_buf[14]);
            twist.angular.y=int(rcv_buf[15]<<24|rcv_buf[16]<<16|rcv_buf[17]<<8|rcv_buf[18]);
            ROS_INFO("back Leftpos=%.2f Rightpos=%.2f",twist.angular.x,twist.angular.y);
       }

        socket_pub.publish(twist);			//发布
        bzero(rcv_buf,sizeof(rcv_buf));
        loop_rate.sleep();
      }
      close(clientSock);

      printf("关闭了一个连接：%d \r\n", clientSock);
    }
  
    close(serverSock);
    printf("关闭了服务：%d \r\n", serverSock);
    return 0;
}


int SocketServer_Mode::Send(unsigned char *send_buf,int len)
{
	
    if(write(clientSock,send_buf,len)==-1){
        printf("Send error!\n");
        return -1;
    }         
    return 0;
 }

 
void SocketServer_Mode::cmdVelocityCallback(const geometry_msgs::Twist &twist)
{
  unsigned char sendbuf[10];
  short w=(short)(twist.angular.z*10000);
  short v=(short)(twist.linear.x*1000);

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
  SocketServer_Mode::Send(sendbuf,10);
  
  ROS_INFO("cmdV=%.2f cmdW=%.3f", twist.linear.x,twist.angular.z);

}
 



