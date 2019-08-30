#ifndef _MOBILE_BASE_
#define _MOBILE_BASE_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>

namespace Ginger
{
    const float cos60 = 0.5;
    const float sin60 = 0.8660254;

class OmniOdometry {
public:
    const float MIN_LIMIT=0.0007;
    //const float WHEEL_RADIUS=0.08;
    const float WHEEL_RADIUS=0.075;
    //const float BASE_RADIUS=0.234;
    const float BASE_RADIUS=0.2184;//fix for linear distance;
    //const float BASE_RADIUS=0.218109367;//fix for rotation;0.001332509476
    const float WARP_VALUE=33.510*2;//44.680428851; //33.510
    const float MAX_LIMIT=WARP_VALUE/2;
    enum{
        W_LEFT=0,
        W_RIGHT=1,
        W_BACK=2,
    };
    
    OmniOdometry():x(0),y(0),theta(0),linear_vel(0),angular_vel(0),initialized(false),counter(0) {

    }
    ~OmniOdometry() {
    }
    void update_joints(const sensor_msgs::JointStateConstPtr &js);
    void get_odom(geometry_msgs::TransformStamped &odom_trans,nav_msgs::Odometry &odom);
    void set_odom(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
private:
    sensor_msgs::JointState _js;
    ros::Time _ts;
    int counter;
    bool initialized;
    double x;   //positive front;
    double y;
    double theta; //initial with 0;
    double linear_vel;
    double angular_vel;
    double position[3];
    double position_now[3];
    double velocity[3];

};

class MobileBase
{
public:
    MobileBase(float base_radius, float wheel_radius);
    ~MobileBase();

    void baseInitialize();

    void getOdom();
    void teleopCallback(const geometry_msgs::TwistConstPtr &cmd_vel); 
    void jointsCallback(const sensor_msgs::JointStateConstPtr &jointsState); 
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

private:
    float _base_radius;
    float _wheel_radius;
    ros::NodeHandle _nh;
    ros::Publisher _joint_pub;
    ros::Subscriber _teleop_sub;
    ros::Subscriber _joint_sub;
    ros::Subscriber _pose_sub;
    sensor_msgs::JointState _js;
    OmniOdometry _odom;
    ros::Publisher _odom_pub;
    tf::TransformBroadcaster _odom_broadcaster;  

};
}


#endif
