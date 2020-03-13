/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Atom-Robotics Corporation. All rights reserved.
 *  Author: niuyunzhu
 *  Last Modify: 2019.10.24
 *  Description: Saber Measure Mode get data stream tools  on ROS/Linux
 *--------------------------------------------------------------------------------------------*/
#include "../saber_ros_inc/saber_tool.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "../saber_ros_inc/saber_macro.h"
#include "../saber_ros_inc/saber_serial.h"
#include "../saber_ros_inc/saber_protocol.h"

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberAlign
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.24
 *  Description: Saber measure data packet align on the serial stream
 *  Parameter:   nFD = serialport file desriptor, used by system call ,such as read,write
 *  Return:      32bit int = data packet length(bytes)           
 *--------------------------------------------------------------------------------------------*/
int SaberAlign(unsigned char nFD)
{
    unsigned char tmp = 0;
    //unsigned char tmp1 = 0;
    unsigned int uLen = 0;
    int packLength = 0;
    unsigned int tmp_i = 0;
    fprintf(stderr, "SaberAlign.\n");
    int n = 0;
    while (1)
    {
        uLen = read(nFD, &tmp, 1);
        fprintf(stderr, "len:%u data:%02x\n", uLen, tmp);
        if (0 == uLen)
            n++;

        if (n > 10)
        {
            fprintf(stderr, "no data received,quit\n");
            return -1;
        }

        if (tmp == 0x6D)
        {
            uLen = read(nFD, &tmp, 1);
            if (tmp == 0x41)
            {

                uLen = read(nFD, &tmp_i, 4);
#ifndef MCU_ON
                printf("Head :%x\n", tmp_i);
#endif
                if (tmp_i == 0x8106FF78)
                {
                    uLen = read(nFD, &tmp, 1);
                    packLength = tmp;
#ifndef MCU_ON
                    printf("Len :%d\n", packLength);
#endif
                    break;
                }
            }
        }
    }
#ifndef MCU_ON
    //printf("Saber find packet head\n");
#endif
    uLen = 0;
    return packLength;
}

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberGetFrame
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.24
 *  Description: Get a frame of Saber measure data packet
 *  Parameter:   nFD = serialport file desriptor, used by system call ,such as read,write
 *               tmpBuf = buffer store the frame of Saber measure data packet 
 *               frameLen = data packet length(bytes)
 *  Return:      32bit int =  data packet length(bytes) equal frameLen usually          
 *--------------------------------------------------------------------------------------------*/
int SaberGetFrame(unsigned char nFD, unsigned char *tmpBuf, int frameLen)
{
    int pkgLen = frameLen;
    int uLen = 0;
    int tLen = 0;
    memset(tmpBuf, 0, pkgLen);
    while (uLen < pkgLen)
    {
        tLen = read(nFD, tmpBuf + uLen, pkgLen - uLen);
        uLen += tLen;
    }
    return uLen;
}
/*---------------------------------------------------------------------------------------------
 *  Function:    SaberValidFrame
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.23
 *  Description: Valid a frame of Saber measure data packet
 *  Parameter:   tmpBuf = buffer store the frame of Saber measure data packet 
 *               frameLen = data packet length(bytes)
 *  Return:      32bit int =  data packet length(bytes) equal frameLen usually          
 *--------------------------------------------------------------------------------------------*/
bool SaberValidFrame(unsigned char *tmpBuf, int frameLen)
{
    return (tmpBuf[0] == 0x41) &
           (tmpBuf[1] == 0x78) &
           (tmpBuf[2] == 0xFF) &
           (tmpBuf[frameLen - 1] == 0x6D);
}

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberFillFrameHead
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.23
 *  Description: Fill the frame head of Saber measure data packet losted by SaberAlign, using couple of  
 *               SaberAlign(unsigned char nFD),such as:
 *               
 *               SaberAlign(nFD);
 *               SaberFillFrameHead(dataBuf);
 *                               
 *  Parameter:   tmpBuf = buffer store the frame of Saber measure data packet 
 *               frameLen = data packet length(bytes)
 *  Return:      32bit int =  data packet length(bytes) equal frameLen usually          
 *--------------------------------------------------------------------------------------------*/
int SaberFillFrameHead(unsigned char *tmpBuf)
{
    tmpBuf[0] = 0x41;
    tmpBuf[1] = 0x78;
    tmpBuf[2] = 0xFF;
    tmpBuf[3] = 0x06;
    tmpBuf[4] = 0x81;
    return 0;
}

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberParserDataPacket
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.23
 *  Description: Parser a frame of Saber measure data packet
 *  Parameter:   saberDataHandle = data struct of all Saber measure data type,such as, linear_acc... 
 *               pBuffer = data buffer
 *               dataLen = data payload length(bytes)
 *               fpLog = a log file ,optional
 *  Return:      nothing        
 *--------------------------------------------------------------------------------------------*/

void SaberParserDataPacket(SaberData *saberDataHandle, u8 *pBuffer, u16 dataLen, FILE *fpLog)
{
    u16 PID = 0;
    u8 *pData = pBuffer;
    u8 index = 0;
    u8 pl = 0;

    //reset saberDataHandle
    memset(saberDataHandle, 0, sizeof(saberDataHandle));
    //printf("\n");
    while (index < dataLen)
    {
        PID = ((*((u16 *)(pData + index))) & 0x7fff);
        pl = *(pData + index + 2);

        // printf("0x%x \n",PID);
        if (PID == (SESSION_NAME_TEMPERATURE))
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->temperature.data, pData + index, PL_TEMPERTURE);
            saberDataHandle->temperature.dataID = PID;
            saberDataHandle->temperature.dataLen = pl;
            //printf(" *** temperature:\t%11.4f *** \n", saberDataHandle->temperature.data);

            index += PL_TEMPERTURE;
        }
        else if (PID == (SESSION_NAME_RAW_ACC))
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accRawData.accX, pData + index, PL_RAW_DATA);
            saberDataHandle->accRawData.dataID = PID;
            saberDataHandle->accRawData.dataLen = pl;

            index += PL_RAW_DATA;
        }
        else if (PID == SESSION_NAME_RAW_GYRO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->gyroRawData.gyroX, pData + index, PL_RAW_DATA);
            saberDataHandle->gyroRawData.dataID = PID;
            saberDataHandle->gyroRawData.dataLen = pl;
            index += PL_RAW_DATA;
        }
        else if (PID == SESSION_NAME_RAW_MAG)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->magRawData.magX, pData + index, PL_RAW_DATA);
            saberDataHandle->magRawData.dataID = PID;
            saberDataHandle->magRawData.dataLen = pl;
            index += PL_RAW_DATA;
        }
        else if (PID == SESSION_NAME_CAL_ACC)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accCal.accX, pData + index, PL_CAL_DATA);
            saberDataHandle->accCal.dataID = PID;
            saberDataHandle->accCal.dataLen = pl;
            index += PL_CAL_DATA;
            if (fpLog != NULL)
                fprintf(fpLog, " *** accCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accCal.accX, saberDataHandle->accCal.accY, saberDataHandle->accCal.accZ);
        }
        else if (PID == SESSION_NAME_CAL_GYRO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->gyroCal.gyroX, pData + index, PL_CAL_DATA);

            saberDataHandle->gyroCal.dataID = PID;
            saberDataHandle->gyroCal.dataLen = pl;
            index += PL_CAL_DATA;
            if (fpLog != NULL)
                fprintf(fpLog, " *** gyroCal:    \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->gyroCal.gyroX, saberDataHandle->gyroCal.gyroY, saberDataHandle->gyroCal.gyroZ);
        }
        else if (PID == SESSION_NAME_CAL_MAG)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->magCal.magX, pData + index, PL_CAL_DATA);
            saberDataHandle->magCal.dataID = PID;
            saberDataHandle->magCal.dataLen = pl;
            index += PL_CAL_DATA;

            //printf(" *** magCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->magCal.magX, saberDataHandle->magCal.magY, saberDataHandle->magCal.magZ);
        }
        else if (PID == SESSION_NAME_KAL_ACC)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accKal.accX, pData + index, PL_KAL_DATA);
            saberDataHandle->accKal.dataID = PID;
            saberDataHandle->accKal.dataLen = pl;
            index += PL_KAL_DATA;
            if (fpLog != NULL)
                fprintf(fpLog, " *** accKal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accKal.accX, saberDataHandle->accKal.accY, saberDataHandle->accKal.accZ);
            fprintf(stderr, " *** accKal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accKal.accX, saberDataHandle->accKal.accY, saberDataHandle->accKal.accZ);
        }
        else if (PID == SESSION_NAME_KAL_GYRO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->gyroKal.gyroX, pData + index, PL_KAL_DATA);
            saberDataHandle->gyroKal.dataID = PID;
            saberDataHandle->gyroKal.dataLen = pl;
            index += PL_KAL_DATA;
        }
        else if (PID == SESSION_NAME_KAL_MAG)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->magKal.magX, pData + index, PL_KAL_DATA);
            saberDataHandle->magKal.dataID = PID;
            saberDataHandle->magKal.dataLen = pl;
            index += PL_KAL_DATA;
        }
        //////////////////////////
        else if (PID == SESSION_NAME_QUAT)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->quat.Q0.uint_x, pData + index, PL_QUAT_EULER);
            saberDataHandle->quat.dataID = PID;
            saberDataHandle->quat.dataLen = pl;
            index += PL_QUAT_EULER;
            if (fpLog != NULL)
                fprintf(fpLog, " *** quat :      \t%11.4f, %11.4f, %11.4f, %11.4f *** \n", saberDataHandle->quat.Q0.float_x, saberDataHandle->quat.Q1.float_x, saberDataHandle->quat.Q2.float_x, saberDataHandle->quat.Q3.float_x);
        }
        else if (PID == SESSION_NAME_EULER)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->euler.roll, pData + index, PL_QUAT_EULER);
            saberDataHandle->euler.dataID = PID;
            saberDataHandle->euler.dataLen = pl;
            index += PL_QUAT_EULER;

            // printf(" *** euler:      \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->euler.roll, saberDataHandle->euler.pitch, saberDataHandle->euler.yaw);
        }

        else if (PID == SESSION_NAME_ROTATION_M)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->romatix.a, pData + index, PL_MATERIX);
            saberDataHandle->romatix.dataID = PID;
            saberDataHandle->romatix.dataLen = pl;
            index += PL_MATERIX;
        }

        else if (PID == SESSION_NAME_LINEAR_ACC)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accLinear.accX, pData + index, PL_LINEAR_ACC_DATA);
            saberDataHandle->accLinear.dataID = PID;
            saberDataHandle->accLinear.dataLen = pl;
            index += PL_LINEAR_ACC_DATA;
            if (fpLog != NULL)
                fprintf(fpLog, " *** lin_acc:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accLinear.accX, saberDataHandle->accLinear.accY, saberDataHandle->accLinear.accZ);
        }

        else if (PID == SESSION_NAME_DELTA_T)
        {
            //Ignore pid and pl
            index += 3;
            memcpy(&saberDataHandle->dt.DT, pData + index, PL_DT_DATA);

            saberDataHandle->dt.dataID = PID;
            saberDataHandle->dt.dataLen = pl;
            index += PL_DT_DATA;
        }

        else if (PID == SESSION_NAME_OS_TIME)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->tick.OS_Time_ms, pData + index, PL_OS_REFERENCE_TIME - 2); //first 4 bytes are miliseconds
            saberDataHandle->tick.OS_Time_ms = *((u32 *)(pData + index));
            saberDataHandle->tick.OS_Time_us = *((u16 *)(pData + index + 4));

            saberDataHandle->tick.dataID = PID;
            saberDataHandle->tick.dataLen = pl;
            index += PL_OS_REFERENCE_TIME;
        }
        else if (PID == SESSION_NAME_STATUS_WORD)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->status.status, pData + index, PL_STATUS);
            saberDataHandle->status.dataID = PID;
            saberDataHandle->status.dataLen = pl;
            index += PL_STATUS;
        }
        else if (PID == SESSION_NAME_PACKET_COUNTER)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->packetCounter.packerCounter, pData + index, PL_PACKET_NUMBER);
            saberDataHandle->packetCounter.dataID = PID;
            saberDataHandle->packetCounter.dataLen = pl;
            index += PL_PACKET_NUMBER;

            if (fpLog != NULL)
                fprintf(fpLog, " *** packet_count:  %d, *** \n", saberDataHandle->packetCounter.packerCounter);
        }
    }
}