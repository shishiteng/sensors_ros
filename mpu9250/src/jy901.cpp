#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_datatypes.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>

static int ret;
static int fd;

#define BAUD 115200 //9600 //115200 for JY61 ,9600 for others

unsigned long long nanosec()
{
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start); //有4组稍微大于7或者小于3的
    //clock_gettime(CLOCK_MONOTONIC, &time_start); //有很多组间隔小于1的
    //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time_start); //很多组大于10的
    //clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time_start);
    unsigned long long ns = (unsigned long long)(time_start.tv_sec * 1000000000 + time_start.tv_nsec);

    return ns;
}

int uart_open(int fd, const char *pathname)
{
    fd = open(pathname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (-1);
    }
    else
        printf("open %s success!\n", pathname);
    if (isatty(STDIN_FILENO) == 0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    switch (nEvent)
    {
    case 'o':
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'e':
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        break;
    }

    /*设置波特率*/

    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);

    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    return 0;
}

int send_data(int fd, char *send_buffer, int length)
{
    length = write(fd, send_buffer, length * sizeof(unsigned char));
    return length;
}

int recv_data(int fd, char *recv_buffer, int length)
{
    length = read(fd, recv_buffer, length);
    return length;
}

float a[3], w[3], Angle[3], h[3];
int height;
int pressure;
void ParseData(char chr)
{
    static char chrBuf[100];
    static unsigned char chrCnt = 0;
    signed short sData[4];
    unsigned char i;

    time_t now;
    chrBuf[chrCnt++] = chr;
    if (chrCnt < 44)
        return;

    if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50))
    {
        printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
        memcpy(&chrBuf[0], &chrBuf[1], 10);
        chrCnt--;
        return;
    }

    memset(&sData[0], 0, 4);
    memcpy(&sData[0], &chrBuf[2], 8);

    switch (chrBuf[1])
    {
    case 0x51:
        for (i = 0; i < 3; i++)
            a[i] = (float)sData[i] / 32768.0 * 16.0;
        time(&now);
        // printf("\r\nT:%s a:%6.3f %6.3f %6.3f ", asctime(localtime(&now)), a[0], a[1], a[2]);

        break;
    case 0x52:
        for (i = 0; i < 3; i++)
            w[i] = (float)sData[i] / 32768.0 * 2000.0;
        // printf("w:%7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
        break;
    case 0x53:
        for (i = 0; i < 3; i++)
            Angle[i] = (float)sData[i] / 32768.0 * 180.0;
        // printf("A:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);
        break;
    case 0x54:
        for (i = 0; i < 3; i++)
            h[i] = (float)sData[i];
        // printf("h:%4.0f %4.0f %4.0f ", h[0], h[1], h[2]);
        break;
    case 0x56:
        char *P = (char *)&sData[0];
        char *H = (char *)&sData[2];
        pressure = (int)((P[3] & 0xff) << 24) + (int)((P[2] & 0xff) << 16) + (int)((P[1] & 0xff) << 8) + (int)P[0]; //（Pa）
        height = (int)((H[3] & 0xff) << 24) + (int)((H[2] & 0xff) << 16) + (int)((H[1] & 0xff) << 8) + (int)H[0];   // cm
        // printf("height: %d pressure: %d\n", height, pressure);

        break;
    }
    chrCnt = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jy901");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu0", 1);
    ros::Publisher imu_pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu0_pose", 20);
    ros::Publisher pressure_pub = n.advertise<sensor_msgs::FluidPressure>("barometer", 1);

    double sa = 9.8;
    double sg = 3.141592653 / 180.f;

    char r_buf[1024];
    bzero(r_buf, 1024);

    fd = uart_open(fd, "/dev/ttyUSB0"); /*串口号/dev/ttySn,USB口号/dev/ttyUSBn */
    if (fd == -1)
    {
        fprintf(stderr, "uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if (uart_set(fd, BAUD, 8, 'N', 1) == -1)
    {
        fprintf(stderr, "uart set failed!\n");
        exit(EXIT_FAILURE);
    }

    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        int ret = select(fd + 1, &rd, NULL, NULL, NULL);
        if (ret < 0)
        {
            perror("select error!\n");
            continue;
        }
        else if (ret == 0)
        {
            perror("time out!\n");
            continue;
        }

        if (FD_ISSET(fd, &rd))
        {
            ret = recv_data(fd, r_buf, 100);
            if (ret == -1)
            {
                fprintf(stderr, "uart read failed!\n");
                exit(EXIT_FAILURE);
            }
            for (int i = 0; i < ret; i++)
            {
                // printf("%2X ", r_buf[i]);
                // fprintf(fp, "%2X ", r_buf[i]);
                ParseData(r_buf[i]);
            }
            if (ret > 0)
            {
                // imu data
                //printf("%lf _ %f %f %f %f %f %f\n", (double)nanosec() / 1000000000.f, w[0], w[1], w[2], a[0], a[1], a[2]);

                sensor_msgs::Imu imu_msg;
                imu_msg.header.frame_id = "imu";
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.angular_velocity.x = w[0] * sg;
                imu_msg.angular_velocity.y = w[1] * sg;
                imu_msg.angular_velocity.z = w[2] * sg;
                imu_msg.linear_acceleration.x = a[0] * sa;
                imu_msg.linear_acceleration.y = a[1] * sa;
                imu_msg.linear_acceleration.z = a[2] * sa;
                imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(Angle[0] * sg, Angle[1] * sg, Angle[2] * sg);

                imu_pub.publish(imu_msg);

                // pose
                geometry_msgs::PoseStamped posestamped;
                posestamped.header = imu_msg.header;
                posestamped.pose.orientation = imu_msg.orientation;
                posestamped.pose.position.x = 0;
                posestamped.pose.position.y = 0;
                posestamped.pose.position.z = 0;
                imu_pose_pub.publish(posestamped);

                // barometer data
                sensor_msgs::FluidPressure pressure_msg;
                pressure_msg.header.stamp = imu_msg.header.stamp;
                pressure_msg.fluid_pressure = (double)pressure / 100.f;
                pressure_pub.publish(pressure_msg);
            }
        }

        // usleep(5000);
        // loop_rate.sleep();
    }

    ret = uart_close(fd);
    if (ret == -1)
    {
        fprintf(stderr, "uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
