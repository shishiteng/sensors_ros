#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

/**************积分***********************/
class IntegrationOdometry
{
public:
  Eigen::Vector3d vel_0,vel_1;
  Eigen::Vector3d gyr_0,gyr_1;
  double dt;
  double sum_dt;
  Eigen::Vector3d delta_p;
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_v;

public:
  IntegrationOdometry() = delete;
  IntegrationOdometry(const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_gyr_0)
    : vel_0{_vel_0}, gyr_0{_gyr_0},sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, 
    delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

  {

  }


  void midPointIntegration(double _dt, 
			   const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_gyr_0,
			   const Eigen::Vector3d &_vel_1, const Eigen::Vector3d &_gyr_1,
			   const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
			   Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v)
  {
    //ROS_INFO("midpoint integration");
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1);
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Vector3d un_vel_0 = delta_q * (_vel_0);
    Vector3d un_vel_1 = result_delta_q * (_vel_1);
    Vector3d un_vel = 0.5 * (un_vel_0 + un_vel_1);
    //cout<< "v:"<<delta_v.transpose()<<endl;
    //cout<< "p:"<<delta_p.transpose()<<endl;
    //cout<< "dt:"<<_dt<<endl;
    result_delta_p = delta_p + delta_v * _dt;
    //cout<< "dp:"<<result_delta_p.transpose()<<endl;
    //result_delta_v = delta_v;
    result_delta_v = un_vel;
  }

  void propagate(double _dt, const Eigen::Vector3d &_vel_1, const Eigen::Vector3d &_gyr_1)
  {
    dt = _dt;
    vel_1 = _vel_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p(Eigen::Vector3d::Zero());
    Vector3d result_delta_v(Eigen::Vector3d::Zero());
    Quaterniond result_delta_q;
#if 0
    printf("propagete_input: t[%.3f]\n v0[%.3f %.3f %.3f] w[%.3f %.3f %.3f]\n v1[%.3f %.3f %.3f] w[%.3f %.3f %.3f]\n",
	   dt,
	   vel_0[0],vel_0[1],vel_0[2],
	   gyr_0[0],gyr_0[1],gyr_0[2],
	   vel_1[0],vel_1[1],vel_1[2],
	   gyr_1[0],gyr_1[1],gyr_1[2]);
#endif
    
    //
    midPointIntegration(_dt, vel_0, gyr_0, _vel_1, _gyr_1, delta_p, delta_q, delta_v,
			result_delta_p, result_delta_q, result_delta_v);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    delta_q.normalize();
    sum_dt += dt;
    vel_0 = vel_1;
    gyr_0 = gyr_1;

    //    printf("propagete_output: delta_p[%.3f %.3f %.3f]\n",
    //	   result_delta_p[0],result_delta_p[1],result_delta_p[2]);
  }
      
};
/*********************************/

//
IntegrationOdometry *pOdomIntegration = NULL;
Vector3d vel_0, gyr_0;
double current_time = -999.0;
bool first_odom = false;
int frame_count;

void processOdom(double dt, const Vector3d &linear_velocity, const Vector3d &angular_velocity)
{
  if (!first_odom) {
    first_odom = true;
    vel_0 = linear_velocity;
    gyr_0 = angular_velocity;
  }

  if (!pOdomIntegration) {
    pOdomIntegration = new IntegrationOdometry(vel_0, gyr_0);
  }

  pOdomIntegration->propagate(dt, linear_velocity, angular_velocity);
  frame_count++;
}


void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  double t = odom_msg->header.stamp.toSec();
  if (current_time < 0)
    current_time = t;
  double dt = t - current_time;
  current_time = t;

  double dx = (double)odom_msg->twist.twist.linear.x;
  double dy = (double)odom_msg->twist.twist.linear.y;
  double dz = (double)odom_msg->twist.twist.linear.z;

  double rx = (double)odom_msg->twist.twist.angular.x;
  double ry = (double)odom_msg->twist.twist.angular.y;
  double rz = (double)odom_msg->twist.twist.angular.z;
  //ROS_INFO("Odom %f, dt: %f, vel: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);
  
  processOdom(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz)); 

  //publish result
  if(pOdomIntegration) {
    //cout<<"p:"<<pOdomIntegration->delta_p.transpose()<<endl;
    std_msgs::Header header = odom_msg->header;
    //header.frame_id = "world";

    nav_msgs::Odometry odometry;
    odometry.header = header;
    //odometry.header.frame_id = "world";
    //odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = (float)pOdomIntegration->delta_p.x();
    odometry.pose.pose.position.y = (float)pOdomIntegration->delta_p.y();
    odometry.pose.pose.position.z = (float)pOdomIntegration->delta_p.z();
    odometry.pose.pose.orientation.x = (float)Quaterniond(pOdomIntegration->delta_q).x();
    odometry.pose.pose.orientation.y = (float)Quaterniond(pOdomIntegration->delta_q).y();
    odometry.pose.pose.orientation.z = (float)Quaterniond(pOdomIntegration->delta_q).z();
    odometry.pose.pose.orientation.w = (float)Quaterniond(pOdomIntegration->delta_q).w();
    pub_odom.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = odometry.pose.pose;

    //path
    
    path.header = header;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    //pose
    pose_stamped.pose.position.x = 0;
    pose_stamped.pose.position.y = 0;
    pose_stamped.pose.position.z = 0;
    pub_pose.publish(pose_stamped);
  }
}

