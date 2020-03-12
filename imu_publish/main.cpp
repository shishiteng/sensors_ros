/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Atom-Robotics Corporation. All rights reserved.
 *  Author: niuyunzhu
 *  Last Modify: 2019.10.20
 *  Description: Saber Demo Project on ROS
 *  Licensed under the Apache License Version 2.0
 *  See https://www.apache.org/licenses/LICENSE-2.0.html for license information.
 *--------------------------------------------------------------------------------------------*/

#include "main.h"
//std include
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#define ROS_ON

//ros include
#ifdef ROS_ON
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>
#endif

//usr include
#include "saber_ros_inc/saber_serial.h"
#include "saber_ros_inc/saber_macro.h"
#include "saber_ros_inc/saber_protocol.h"
#include "saber_ros_inc/saber_config.h"
#include "saber_ros_inc/saber_tool.h"

using namespace std;

//global variant
SaberData saberDataHandle;
ros::Publisher pub, pub_pose;
sensor_msgs::Imu imuMsg;
geometry_msgs::PoseStamped posestamped;

//Warning:Don't use malloc to get buffer,Just use a global array;
unsigned char frmBuf[256] = {0};

//main
int main(int argc, char **argv)
{
    char nFD = 0;
    int packLengthFW = 0;
    int pkgLen = 0;
    int pubCnt = 0;
    FILE *fpLog = NULL;
    u8 ret = 0;
    int seq = 0;
    bool met;
    int cycleCnt = 0;
    int errCnt = 0;

    unsigned char *dataBuf = NULL;
    dataBuf = &frmBuf[0];

#ifndef MCU_ON
    printf("Hello,Saber on ROS!\n");
#endif

#ifdef ROS_ON
    ros::init(argc, argv, "atomC2");
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::Imu>("imu0", 20);
    pub_pose = n.advertise<geometry_msgs::PoseStamped>("imu0_pose", 20);
#endif

    if (dataBuf == NULL)
    {
#ifndef MCU_ON
        printf("Data buffer %d bytes valid fail \n", pkgLen);
#endif
        return -2;
    }

    //step 1: read config,open serialport,get serialport file desriptor
    //be careful to choose file path
    nFD = SaberInitConfig("/home/sst/catkin_ws/src/sensors_ros/imu_publish/config/saber_cfg.json");
    if (nFD < 0)
    {
        printf("saber init failed.\n");
        return -1;
    }

    //step 2: align Saber data frame from the serial stream
    packLengthFW = SaberAlign(nFD);
    if (packLengthFW < 0)
    {
        return -1;
    }

    pkgLen = packLengthFW + SABER_EMPTY_LEN;
    SaberFillFrameHead(dataBuf);

    //step 3:get a whole frame and valid the frame
    SaberGetFrame(nFD, dataBuf + SABER_HEAD_LEN, packLengthFW + SABER_TAIL_LEN);
    printf("0\n");
    while (!SaberValidFrame(dataBuf, pkgLen))
    {
        packLengthFW = SaberAlign(nFD);
        SaberFillFrameHead(dataBuf);
        SaberGetFrame(nFD, dataBuf + SABER_HEAD_LEN, packLengthFW + SABER_TAIL_LEN);
    }

    //get a whole frame and valid the frame by ros::rate,such as 100hz
#ifdef ROS_ON
    ros::Rate r(100);
    while (ros::ok())
#else
    while (1)
#endif
    {
        //SaberFillFrameHead(dataBuf);
        SaberGetFrame(nFD, dataBuf, packLengthFW + SABER_EMPTY_LEN);

        while (!SaberValidFrame(dataBuf, pkgLen))
        {
            packLengthFW = SaberAlign(nFD);
            SaberFillFrameHead(dataBuf);
            SaberGetFrame(nFD, dataBuf + SABER_HEAD_LEN, packLengthFW + SABER_TAIL_LEN);

            // 5s
            if (errCnt++ > 5000000)
            {
                printf("no data!\n");
                return -1;
            }
            usleep(1000);
        }
        //step 4:parser a whole frame to generate ros publish data
        SaberParserDataPacket(&saberDataHandle, &dataBuf[SABER_HEAD_LEN], packLengthFW, fpLog);
#ifdef ROS_ON
        imuMsg.linear_acceleration.x = saberDataHandle.accLinear.accX;
        imuMsg.linear_acceleration.y = saberDataHandle.accLinear.accY;
        imuMsg.linear_acceleration.z = saberDataHandle.accLinear.accZ;

        imuMsg.angular_velocity.x = saberDataHandle.gyroCal.gyroX * DEG_RAD;
        imuMsg.angular_velocity.y = saberDataHandle.gyroCal.gyroY * DEG_RAD;
        imuMsg.angular_velocity.z = saberDataHandle.gyroCal.gyroZ * DEG_RAD;

        double norm = sqrt(saberDataHandle.quat.Q0.float_x * saberDataHandle.quat.Q0.float_x +
                           saberDataHandle.quat.Q1.float_x * saberDataHandle.quat.Q1.float_x +
                           saberDataHandle.quat.Q2.float_x * saberDataHandle.quat.Q2.float_x +
                           saberDataHandle.quat.Q3.float_x * saberDataHandle.quat.Q3.float_x);

        imuMsg.orientation.x = saberDataHandle.quat.Q0.float_x / norm;
        imuMsg.orientation.y = saberDataHandle.quat.Q1.float_x / norm;
        imuMsg.orientation.z = saberDataHandle.quat.Q2.float_x / norm;
        imuMsg.orientation.w = saberDataHandle.quat.Q3.float_x / norm;

        imuMsg.header.stamp = ros::Time::now();
        imuMsg.header.frame_id = "imu_link";
        imuMsg.header.seq = seq;
        seq = seq + 1;
        pub.publish(imuMsg);

        posestamped.header.stamp = imuMsg.header.stamp;
        posestamped.header.frame_id = "imu_link";
        posestamped.pose.position.x = 0;
        posestamped.pose.position.y = 0;
        posestamped.pose.position.z = 0;
        posestamped.pose.orientation = imuMsg.orientation;
        pub_pose.publish(posestamped);

        pubCnt++;
        ROS_INFO(" *** publish_count: %d, *** \n", pubCnt);

        ros::spinOnce();
        met = r.sleep();
#endif
    }
    if (nFD > 0)
    {
        Saber_CloseSerialPort(nFD);
    }
    if (fpLog != NULL)
        fclose(fpLog);
#ifndef MCU_ON
    printf("%d Frames record,%d  Interrupt Frames\n", cycleCnt - 1, errCnt);
#endif
    return 0;
}
