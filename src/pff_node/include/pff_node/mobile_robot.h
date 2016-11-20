/* Copyright 2016 ks */

#ifndef PFF_NODE_MOBILE_ROBOT_H_
#define PFF_NODE_MOBILE_ROBOT_H_

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"

namespace pff
{
class MobileRobot
{
public:
  static constexpr double kWheelDiameter = 0.1;
  static constexpr double kMetersToWheelRad(double val)
  {
    // @TODO(ks): need correct conversion
    return val * kWheelDiameter * 0.1;
  };
  MobileRobot();
  ~MobileRobot();
  void init(ros::NodeHandle &nh);
  void publishTf();
  void publishWheelTf();
  void setVelocity(const tf::Vector3 &vel, bool update = true)
  {
    vel_ = vel;
    vel_.setY(0);  // differential can't strafe
    if (update)
    {
      last_time_command_received_ = ros::Time::now();
    }
  }
  void stop()
  {
    setVelocity(tf::Vector3(0, 0, 0), false);
  }
  inline tf::Vector3 getPosition()
  {
    return pos_;
  }

private:
  ros::NodeHandle *nh_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_joint_states_;       ///< joint_states topic publisher
  tf::Vector3 pos_;                       ///< robot base position
  tf::Vector3 vel_;                       ///< robot base velocity
  sensor_msgs::JointState joint_states_;  ///< joint_states
  ros::Time last_time_;
  ros::Time last_time_command_received_;
  tf::TransformBroadcaster *odom_broadcaster_;
};

MobileRobot::MobileRobot()
{
  pos_.setZero();
  vel_.setZero();
  joint_states_.name.resize(2);
  joint_states_.name[0] = "/joint_wheel_left";
  joint_states_.name[1] = "/joint_wheel_right";
  joint_states_.position.resize(2);
  joint_states_.header.frame_id = "/base_link";
}

MobileRobot::~MobileRobot()
{
  delete odom_broadcaster_;
}

void MobileRobot::init(ros::NodeHandle &nh)
{
  nh_ = &nh;
  last_time_ = ros::Time::now();
  last_time_command_received_ = ros::Time::now();
  odom_broadcaster_ = new tf::TransformBroadcaster();
  pub_joint_states_ = nh_->advertise<sensor_msgs::JointState>("/joint_states", 1);
  pub_odom_ = nh_->advertise<nav_msgs::Odometry>("/odom", 50);
}

void MobileRobot::publishWheelTf()
{
  joint_states_.header.stamp = ros::Time::now();
  joint_states_.position[0] = kMetersToWheelRad(vel_.x() + vel_.z());
  joint_states_.position[1] = kMetersToWheelRad(vel_.x() - vel_.z());
  pub_joint_states_.publish(joint_states_);
}

void MobileRobot::publishTf()
{
  ros::Time current_time = ros::Time::now();

  // start slowing down if velocity command expired
  double dt_cmd = (current_time - last_time_command_received_).toSec();
  if (dt_cmd > 0.5 && !vel_.isZero())
  {
    vel_ *= 0.02;
    if (dt_cmd > 3.0)
    {
      stop();
    }
  }

  double dt = (current_time - last_time_).toSec();
  double delta_x = (vel_.x() * cos(pos_.z()) - vel_.y() * sin(pos_.z())) * dt;
  double delta_y = (vel_.x() * sin(pos_.z()) + vel_.y() * cos(pos_.z())) * dt;
  double delta_th = vel_.z() * dt;

  pos_.setX(pos_.x() + delta_x);
  pos_.setY(pos_.y() + delta_y);
  pos_.setZ(pos_.z() + delta_th);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_.z());

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = pos_.x();
  odom_trans.transform.translation.y = pos_.y();
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster_->sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = pos_.x();
  odom.pose.pose.position.y = pos_.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vel_.x();
  odom.twist.twist.linear.y = vel_.y();
  odom.twist.twist.angular.z = vel_.z();

  // publish odom
  pub_odom_.publish(odom);

  // calculate and publish wheel transforms (non-functional)
  publishWheelTf();

  last_time_ = current_time;
}

}  // namespace pff

#endif  // PFF_NODE_MOBILE_ROBOT_H_
