#include "ginger_chassis/MobileBase.h"
#include "boost/bind.hpp"

namespace Ginger
{
MobileBase::MobileBase(float base_radius, float wheel_radius){
    _base_radius = base_radius;
    _wheel_radius = wheel_radius;
}

MobileBase::~MobileBase()
{
    std::cout << "MobileBase destroy" << std::endl;
}

void MobileBase::baseInitialize()
{
    //ros::init(argc, argv, "MobileBase");
    _joint_pub = _nh.advertise<sensor_msgs::JointState>("mobile/joint_state", 1000);
    _odom_pub = _nh.advertise<nav_msgs::Odometry>("odom", 1000);
    _teleop_sub = _nh.subscribe<geometry_msgs::Twist>("ginger_teleop/cmd_vel", 1000, boost::bind(&MobileBase::teleopCallback, this, _1));
    _joint_sub = _nh.subscribe<sensor_msgs::JointState>("/xr_1/joint_state", 1000, boost::bind(&MobileBase::jointsCallback, this, _1));
//do not modify odom
//    _pose_sub = _nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000, boost::bind(&MobileBase::poseCallback, this, _1));
    
    _js.header.seq = 0;
    _js.header.frame_id = "wheel_link";
    _js.name.push_back("wheel_back");
    _js.name.push_back("wheel_right");
    _js.name.push_back("wheel_left");
}

void MobileBase::jointsCallback(const sensor_msgs::JointStateConstPtr &jointsState)
{
  _odom.update_joints(jointsState);

  //myodom.update_ooints()
}

void MobileBase::teleopCallback(const geometry_msgs::TwistConstPtr &cmd_vel)
{
    float vx = cmd_vel->linear.x;
    float vy = cmd_vel->linear.y;
    float vth = cmd_vel->angular.z;
        
    float vb = -vy + vth * _base_radius;
    float vr = vx * sin60 + vy * cos60 + vth * _base_radius;
    float vl = -vx * sin60 + vy * cos60 + vth * _base_radius;

    _js.velocity.push_back(vb/_wheel_radius);
    _js.velocity.push_back(vr/_wheel_radius);
    _js.velocity.push_back(vl/_wheel_radius);
    _js.header.seq = _js.header.seq + 1;
    _js.header.stamp = ros::Time::now();

    _joint_pub.publish(_js);
    _js.velocity.clear();

}

void MobileBase::getOdom()
{   
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    memset(&odom_trans,0,sizeof(odom_trans));
    memset(&odom,0,sizeof(odom));
    _odom.get_odom(odom_trans, odom);
    _odom_broadcaster.sendTransform(odom_trans);
    _odom_pub.publish(odom);
}

void MobileBase::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    _odom.set_odom(pose);
}
void OmniOdometry::set_odom(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    theta=tf::getYaw(pose->pose.pose.orientation);
    x=pose->pose.pose.position.x;
    y=pose->pose.pose.position.y;

}
void OmniOdometry::get_odom(geometry_msgs::TransformStamped &odom_trans,\
				nav_msgs::Odometry &odom) {
    ros::Time cur_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    //geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = cur_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //_odom_broadcaster.sendTransform(odom_trans);

    //nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = angular_vel;
    //_odom_pub.publish(odom);

}

void OmniOdometry::update_joints(const sensor_msgs::JointStateConstPtr &js) {
    if(js->position.size()!=js->name.size()||js->name.size()!=js->velocity.size()) {
        ROS_INFO("the size of name do not match with position or velocity %ld,%ld,%ld",\
                    js->name.size(), js->position.size(), js->velocity.size());
        return;
    }

    for(int idx = 0; idx < js->name.size(); idx++) {
        if(js->name[idx]=="wheel_left") {
            position_now[W_LEFT] = js->position[idx];
            velocity[W_LEFT] = js->velocity[idx];
        } else if(js->name[idx]=="wheel_right") {
            position_now[W_RIGHT] = js->position[idx];
            velocity[W_RIGHT] = js->velocity[idx];
        } else if(js->name[idx]=="wheel_back") {
            position_now[W_BACK] = js->position[idx];
            velocity[W_BACK] = js->velocity[idx];
        }
    }

    if(!initialized) {
        position[W_LEFT]=position_now[W_LEFT];
        position[W_RIGHT]=position_now[W_RIGHT];
        position[W_BACK]=position_now[W_BACK];
	counter++;
	if(counter > 500)
	    initialized=true;
	return;
    }

    angular_vel = velocity[W_BACK]*WHEEL_RADIUS/BASE_RADIUS;
    linear_vel = ((velocity[W_RIGHT]-velocity[W_LEFT])*WHEEL_RADIUS)/(2*sin60);

    double diff[] = {position_now[0] - position[0],\
                        position_now[1] - position[1],\
                        position_now[2] - position[2]};
    
    for(int idx = 0; idx < (sizeof(diff)/sizeof(diff[0])); idx++) {
        if(fabs(diff[idx]) > MAX_LIMIT) {
            if(diff[idx] > 0) {
                diff[idx]=fmod((diff[idx] - WARP_VALUE) , WARP_VALUE);
            } else {
                diff[idx]=fmod((diff[idx] + WARP_VALUE) , WARP_VALUE);
            }
        }
    }
#if ZERO_Y_SPEED
    if(fabs(diff[W_BACK]) > MIN_LIMIT) {
        for(int idx = 0; idx < (sizeof(diff)/sizeof(diff[0])); idx++) {
            diff[idx]=diff[idx]*WHEEL_RADIUS;
        }
        //TODO tune choose Dth1;
        double L1=(diff[W_RIGHT]-diff[W_BACK])*BASE_RADIUS/(diff[W_BACK]*sin60);
        double L2=(diff[W_BACK]-diff[W_LEFT])*BASE_RADIUS/(diff[W_BACK]*sin60);
        double LA=(diff[W_RIGHT]-diff[W_LEFT])*BASE_RADIUS/(2*diff[W_BACK]*sin60);
        double Dy1=(diff[W_LEFT]-diff[W_BACK])/(-sin60);
        double Dy2=(diff[W_RIGHT]-diff[W_BACK])/(sin60);
        double DyA=(diff[W_RIGHT]-diff[W_LEFT])/(2*sin60);
        double Dth1=diff[W_BACK]/BASE_RADIUS;
        double Dth2=(diff[W_LEFT]+diff[W_RIGHT])/(2*BASE_RADIUS);
        double DthA=(Dth1+Dth2)/2;


        x=DyA*cos(theta)+x;
        y=DyA*sin(theta)+y;
        theta=theta+Dth1;
//        theta=fmod(theta+DthA+2*M_PI, 2*M_PI);
        position[W_LEFT]=position_now[W_LEFT];
        position[W_RIGHT]=position_now[W_RIGHT];
        position[W_BACK]=position_now[W_BACK];
//        ROS_INFO("Dth1 %f, Dth2 %f, DthA %f, theta %f", Dth1, Dth2, DthA, theta);
        
    } else if(fabs(diff[W_LEFT]) > MIN_LIMIT || \
        fabs(diff[W_RIGHT]) > MIN_LIMIT) {
        for(int idx = 0; idx < (sizeof(diff)/sizeof(diff[0])); idx++) {
            diff[idx]=diff[idx]*WHEEL_RADIUS;
        }

        double Dy1=(diff[W_LEFT]-diff[W_BACK])/(-sin60);
        double Dy2=(diff[W_RIGHT]-diff[W_BACK])/(sin60);
        double DyA=(diff[W_RIGHT]-diff[W_LEFT])/(2*sin60);
        x=DyA*cos(theta)+x;
        y=DyA*sin(theta)+y;
        position[W_LEFT]=position_now[W_LEFT];
        position[W_RIGHT]=position_now[W_RIGHT];
    }
#else
if(fabs(diff[W_LEFT]) > MIN_LIMIT || \
        fabs(diff[W_RIGHT]) > MIN_LIMIT || \
	fabs(diff[W_BACK]) > MIN_LIMIT) {
        for(int idx = 0; idx < (sizeof(diff)/sizeof(diff[0])); idx++) {
            diff[idx]=diff[idx]*WHEEL_RADIUS;
        }

        double Dth1=(diff[W_LEFT]+diff[W_RIGHT]+diff[W_BACK])/(BASE_RADIUS*3);
        double Dx=(diff[W_RIGHT]-diff[W_LEFT])/(2*sin60);
        double Dy=(diff[W_RIGHT]+diff[W_LEFT]-2*diff[W_BACK])/(3);

        x=Dx*cos(theta)-Dy*sin(theta)+x;
        y=Dx*sin(theta)+Dy*cos(theta)+y;
        theta=theta+Dth1;
        position[W_LEFT]=position_now[W_LEFT];
        position[W_RIGHT]=position_now[W_RIGHT];
        position[W_BACK]=position_now[W_BACK];
    }

#endif
//    ROS_INFO("x %f, y %f, theta %f", x, y, theta);
}

}
