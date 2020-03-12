//
// Created by atom on 2019/10/22.
//

#ifndef SABER_TOOL_H
#define SABER_TOOL_H

#include <stdio.h>
#include "../saber_ros_inc/saber_macro.h"
#include "../saber_ros_inc/saber_serial.h"
#include "../saber_ros_inc/saber_protocol.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float data;
}SaberData_TEMP_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve0;
    short accX;
    short accY;
    short accZ;
    short gyroX;
    short gyroY;
    short gyroZ;
    short magX;
    short magY;
    short magZ;
    u16 reserve1;

}SaberData_RAW_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float accX;
    float accY;
    float accZ;
}SaberData_CAL_ACC_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float gyroX;
    float gyroY;
    float gyroZ;
}SaberData_CAL_GYRO_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float magX;
    float magY;
    float magZ;
}SaberData_CAL_MAG_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float accX;
    float accY;
    float accZ;
}SaberData_KAL_ACC_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float gyroX;
    float gyroY;
    float gyroZ;
}SaberData_KAL_GYRO_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float magX;
    float magY;
    float magZ;
}SaberData_KAL_MAG_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q0;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q1;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q2;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q3;
}SaberData_Quaternion_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float roll;
    float pitch;
    float yaw;
}SaberData_Euler_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float g;
    float h;
    float i;
}SaberData_RoMax_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u32 packerCounter;
}SaberData_PacketCounter_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float DT;
}SaberData_DT_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float ErrorAll;
}SaberData_MAG_EA_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 percent;

}SaberData_MAG_Strength_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u16 OS_Time_us;
    u32 OS_Time_ms;
}SaberData_OS_Time_ms_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u32 status;
}SaberData_Status_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    double longlatX;
    double longlatY;
}SaberData_Position_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float longlatZ;
}SaberData_Altitude_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u32 ms;
    u16 year;
    u8 month;
    u8 day;
    u8 hour;
    u8 minute;
    u8 second;
    u8 flag;
}SaberData_UTCTime_HandleType;





typedef struct
{
    SaberData_TEMP_HandleType temperature;
    SaberData_RAW_HandleType  accRawData;
    SaberData_RAW_HandleType  gyroRawData;
    SaberData_RAW_HandleType  magRawData;
    SaberData_CAL_ACC_HandleType accCal;
    SaberData_CAL_GYRO_HandleType gyroCal;
    SaberData_CAL_MAG_HandleType magCal;
    SaberData_KAL_ACC_HandleType accKal;
    SaberData_KAL_GYRO_HandleType gyroKal;
    SaberData_KAL_MAG_HandleType magKal;
    SaberData_Quaternion_HandleType quat;
    SaberData_Euler_HandleType euler;
    SaberData_RoMax_HandleType romatix;
    SaberData_PacketCounter_HandleType packetCounter;
    SaberData_OS_Time_ms_HandleType tick;
    SaberData_Status_HandleType status;
    SaberData_UTCTime_HandleType UTC_time;
    //SaberData_CpuUsage_HandleTypeDef CpuUsage;
    SaberData_KAL_ACC_HandleType accLinear;
    SaberData_DT_HandleType dt;
    SaberData_MAG_Strength_HandleType magStrength;
    SaberData_MAG_EA_HandleType magEA;
    SaberData_Position_HandleType position;
    SaberData_Altitude_HandleType altitude;

}SaberData;

extern int SaberAlign(unsigned char nFD);
extern int SaberGetFrame(unsigned char nFD, unsigned char * tmpBuf, int frameLen);
extern void SaberParserDataPacket(SaberData *saberDataHandle, u8 *pBuffer, u16 dataLen, FILE *fpLog);
extern bool SaberValidFrame(unsigned char * tmpBuf,int frameLen);
extern int  SaberFillFrameHead(unsigned char * tmpBuf);
#endif //SABER_TOOL_H
