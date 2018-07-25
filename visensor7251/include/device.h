/*
 * device.h
 *
 *  Created on: Apr 28, 2017
 *      Author: vip2
 */

#ifndef DEVICE_H_
#define DEVICE_H_
// cypress usb
#include "cyusb.h"
// opencv
#include "opencv2/opencv.hpp"

#include <iostream>
#include <string>

namespace sn
{
enum imgMode
{
    SBS,  // side by side
    SLAR, // split left and right
};

enum resolution_mode
{
    HD2K,
    HD1080,
    HD720,
    VGA,  // 640x480
    HVGA, //320x240
};

enum
{
    SRC_IMAGE = 0x1,                //原图
    REMAP_IMAGE = SRC_IMAGE << 1,   // 校正后的图像
    DISPARITY = REMAP_IMAGE << 1,   // 输出视差图
    ERR_CHECK_DIS = DISPARITY << 1, // 视差图加入错误检查
};

struct resolution
{
    int width;
    int height;

    resolution(int w_, int h_)
    {
        width = w_;
        height = h_;
    }
};

struct imuData{
    short gyr_z;
    short gyr_y;
    short gyr_x;
    short invalid0;  // 
    short accel_z;
    short accel_y;
    short accel_x;
    uchar invalid1;
    uchar imu_status;

    short a1;
    short a2;
    short a3;
    short temperature;
    uint64 invalid2; //mag
    
    uint64 imu_timestamp;
    uint64 img_timestamp;
};

// 读取usb数据线程
static void *XferLoop(void *args);

class device
{
  public:
    device(resolution_mode rm = VGA, imgMode mode = SLAR);
    ~device();

    void close();

    // 连接usb设备
    bool init();
    // 暂停fpga数据传输并接收完usb缓存数据
    bool clearData();
    // 设置寄存器
    bool setup(std::string regAddr, std::string regData);
    // 开启采集图像线程
    bool start();

    /****************加入硬件同步，接口扩展，start**************/
    // 配置设备：imu帧率、图像帧率、最大曝光时间、最小曝光时间、当前曝光时间
    bool config(int img_freq,int imu_freq,int max_exposure_time,int min_exposure_time,int cur_exposure_time,int offset_k1,int offset_k2);

    // 设置曝光模式,0-自动曝光，1-固定曝光
    bool setExposureMode(int flag);

    // 设置曝光时间(us)
    bool setExposureTime(double n);

    // 设置增益(倍数)
    bool setGain(int n);

    // 配置imu低通滤波
    bool setDlfpFlag(int acc_dlfp_flag, int gyr_dlfp_flag);

    // 设置imu初始偏置,imu原始数据带有偏置，配置芯片减去偏置量
    bool setImuOffset(int *acc, int *gyr);

    /****************加入硬件同步，接口扩展，end**************/
    

    // 外部调用接口：获得图像数据
    bool QueryFrame(cv::Mat &leftImg, cv::Mat &rightImg, uint64 &imgTimeStamp);
    bool QueryFrame(cv::Mat &leftImg, cv::Mat &rightImg, cv::Mat &dispImg);
    // 外部调用接口：获取imu
    bool readIMU(imuData *imudata);

    // 
    int getImageData(cv::Mat &leftImg, cv::Mat &rightImg, uint64 &imgTimeStamp);
    int getImuData(imuData *imudata);
    
    // 获取remap图像
    bool remapFrame(cv::Mat &leftImg, cv::Mat &rightImg);
    // 获得disparity
    bool disFrame(cv::Mat &dispImg);

    // 外部获取分辨率
    resolution getImageSize();

    // 发送指令
    bool sendCmd(uint Base_addr, uint Data, uint offset = 0x0, uint cmd = 0x0, uint dir = 0x0, uint id = 0x1234);

    // 发送remap配置
    bool uploadRemap(std::string filename, bool left, int bits);

    //
    int receiveData(uchar *buf, int &transferred);

    // 获取数据后分类处理函数
    void parseData(const void *lpData, const long dwSize);

  private:
    // 保存一行原图
    bool writeOneRowImg(const void *data, uint row, unsigned int frameNum);
    // 保存一行remap图
    bool writeOneRowRemapImg(const void *data, uint row);
    // 保存一行disparity图像
    bool writeOneRowDisp(const void *data, uint row);
    // 线程句柄
    pthread_t m_collect_thread;
    // 指针交换
    inline void swapPtr(void **p1, void **p2);
    // 发送数据到device
    bool sendData(void *data, int bytes, uint rowNum, uint cmd, uint id = 0x1234);
    // usb handle
    cyusb_handle *m_device;
    // bulkout
    uint m_bulkout_addr;
    // bulkin
    uint m_bulkin_addr;
    // 两个摄像头的偏移量
    uint m_leftOffset;
    uint m_rightOffset;
    // 分辨率
    uint m_width;
    uint m_height;
    // 模式
    imgMode m_mode;
    // 创建图像Mat和缓存
    cv::Mat m_sbsImage;
    cv::Mat m_sbsImage_cache;
    cv::Mat m_leftImage;
    cv::Mat m_leftImage_cache;
    cv::Mat m_rightImage;
    cv::Mat m_rightImage_cache;
    cv::Mat m_dispImage;
    cv::Mat m_dispImage_cache;
    cv::Mat m_remap_leftImage;
    cv::Mat m_remap_leftImage_cache;
    cv::Mat m_remap_rightImage;
    cv::Mat m_remap_rightImage_cache;
    // 保存对应的指针
    uchar *m_sbsImage_ptr;
    uchar *m_sbsImage_cache_ptr;
    uchar *m_leftImage_ptr;
    uchar *m_leftImage_cache_ptr;
    uchar *m_rightImage_ptr;
    uchar *m_rightImage_cache_ptr;
    uchar *m_dispImage_ptr;
    uchar *m_dispImage_cache_ptr;
    uchar *m_remap_leftImage_ptr;
    uchar *m_remap_leftImage_cache_ptr;
    uchar *m_remap_rightImage_ptr;
    uchar *m_remap_rightImage_cache_ptr;
    // spinlock
    pthread_spinlock_t m_imglock;
    pthread_spinlock_t m_displock;
    pthread_spinlock_t m_remaplock;
    pthread_spinlock_t m_imulock;
    // 解析包后剩下的字节数
    uint m_remainCount;
    uchar *m_cacheInBuf;
    // 每一行对应的字节数
    uint m_sbs_rowBytes;
    uint m_single_rowBytes;
    uint m_disp_rowBytes;
    // 是否存在新数据
    bool m_existNewImg;
    bool m_existNewRemap;
    bool m_existNewDisp;
    bool m_existNewImu;

    // 新添加 imu和图像时间戳
    // 新添加 imu和图像时间戳
    uint64         m_imgTimeStamp;
    uint           m_img_frame;
    uint64         m_imgTimeStamp_cache;
};

} /* namespace sn */

#endif /* DEVICE_H_ */
