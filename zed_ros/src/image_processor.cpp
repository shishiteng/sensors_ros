#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

cv::Ptr<cv::Feature2D> detector_ptr;
int key = -1;
int show_fast = 1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat image = cv_bridge::toCvCopy(msg, "mono8")->image;
    //imshow("image",image);
    //waitKey(1);

    Mat color(image.size(),CV_8UC3);
    cvtColor(image, color, CV_GRAY2RGB);
    //imshow("color",color);
    //waitKey(1);

    
    vector<KeyPoint> new_features(0);
    detector_ptr->detect(image, new_features);
    if(show_fast) {
      for(int i=0;i<new_features.size();i++) {
	circle(color, new_features[i].pt, 3, Scalar(0,0,255), -1); //x
      }
    }
	
    vector<Point2f> gfft_features(0);
    cv::goodFeaturesToTrack(image, gfft_features, 200, 0.01, 20);
    for(int i=0;i<gfft_features.size();i++) {
      circle(color, gfft_features[i], 3, Scalar(0,255,0), -1); //x
    }
      
    imshow("corners",color);
    key = waitKey(1) & 0xffff;
    if(key == 'f')
      show_fast ^= 1;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zed_processor");
  ros::NodeHandle nh;

  detector_ptr = FastFeatureDetector::create(10,true);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("cam0/image_raw", 1, imageCallback);
  
  ros::spin();
  //cv::destroyWindow("view");
}
