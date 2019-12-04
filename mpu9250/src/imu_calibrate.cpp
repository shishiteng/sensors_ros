#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

ros::Publisher pub_;

//温漂补偿、常数噪声补偿、尺度偏差、轴间偏差
int temperature_compensation = 1;
int bias_correction = 1;
int scale_correction = 1;
int axis_alignment = 1;

double sa = 0.000244 * 9.8;
double sg = 0.061035 * 3.141592653 / 180.0;

//校正公式：a = T*K*(a_meas - a_bias - a_tempcompesation)
Mat tempdriftscale_acc_, bias_acc_, scale_acc_, alignment_acc_;
Mat tempdriftscale_gyr_, bias_gyr_, scale_gyr_, alignment_gyr_;

int readConfig(char *config_file)
{
  cout << "---config settings---" << endl;
  FileStorage fsettings(string(config_file), FileStorage::READ);
  if (!fsettings.isOpened())
  {
    fprintf(stderr, "打开配置文件失败:%s\n", config_file);
    return -1;
  }

  fsettings["tempdriftscale_acc"] >> tempdriftscale_acc_;
  fsettings["tempdriftscale_gyr"] >> tempdriftscale_gyr_;

  fsettings["bias_acc"] >> bias_acc_;
  fsettings["bias_gyr"] >> bias_gyr_;

  fsettings["Ka"] >> scale_acc_;
  fsettings["Kg"] >> scale_gyr_;

  fsettings["Ta"] >> alignment_acc_;
  fsettings["Tg"] >> alignment_gyr_;

  cout << " tempdriftscale_acc:\n    " << tempdriftscale_acc_.t() << endl;
  cout << " tempdriftscale_gyr:\n    " << tempdriftscale_gyr_.t() << endl;
  cout << " bias_acc:\n    " << bias_acc_.t() << endl;
  cout << " bias_gyr:\n    " << bias_gyr_.t() << endl;
  cout << " scale_acc:\n"
       << scale_acc_ << endl;
  cout << " scale_gyr:\n"
       << scale_gyr_ << endl;
  cout << " alignment_acc:\n"
       << alignment_acc_ << endl;
  cout << " alignment_gyr:\n"
       << alignment_gyr_ << endl;
  cout << "-----------------\n"
       << endl;

  return 0;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  printf("%lf %.0lf %.0lf %.0lf %.0lf %.0lf %.0lf\n", msg->header.stamp.toSec(),
         msg->angular_velocity.x / sg,
         msg->angular_velocity.y / sg,
         msg->angular_velocity.z / sg,
         msg->linear_acceleration.x / sa,
         msg->linear_acceleration.y / sa,
         msg->linear_acceleration.z / sa);

  //a = TK(a_meas - a_offset -a_tempcompesation)
  double data_acc[3] = {msg->linear_acceleration.x / sa, msg->linear_acceleration.y / sa, msg->linear_acceleration.z / sa};
  double data_gyr[3] = {msg->angular_velocity.x / sg, msg->angular_velocity.y / sg, msg->angular_velocity.z / sg};
  Mat meas_acc(3, 1, CV_64FC1, data_acc);
  Mat meas_gyr(3, 1, CV_64FC1, data_gyr);
  Mat temp_drift_acc = Mat::zeros(3, 1, CV_64FC1); //get_temperature_drift(temperature, tempdriftscale_acc);
  Mat temp_drift_gyr = Mat::zeros(3, 1, CV_64FC1);
  ; //get_temperature_drift(temperature, tempdriftscale_gyr);

  Mat acc = alignment_acc_ * scale_acc_ * (meas_acc - bias_acc_ - temp_drift_acc);
  Mat gyr = alignment_gyr_ * scale_gyr_ * (meas_gyr - bias_gyr_ - temp_drift_gyr);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = msg->header.stamp;
  imu_msg.angular_velocity.x = gyr.at<double>(0);
  imu_msg.angular_velocity.y = gyr.at<double>(1);
  imu_msg.angular_velocity.z = gyr.at<double>(2);
  imu_msg.linear_acceleration.x = acc.at<double>(0);
  imu_msg.linear_acceleration.y = acc.at<double>(1);
  imu_msg.linear_acceleration.z = acc.at<double>(2);
  // imu_msg.orientation_covariance[0] = temperature; //加入温度
  pub_.publish(imu_msg);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    fprintf(stderr, "参数错误,正确形式:\n    rosrun imu_calibrate imu_calibrate [config_file] \n");
    return -1;
  }

  // read config files
  if (readConfig(argv[1]))
  {
    fprintf(stderr, "read config files failed.\n");
    return -1;
  }

  ros::init(argc, argv, "imu_calibrate");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("imu0", 1000, imuCallback);
  pub_ = n.advertise<sensor_msgs::Imu>("imu0_calibrated", 1);

  ros::spin();
  return 0;
}
