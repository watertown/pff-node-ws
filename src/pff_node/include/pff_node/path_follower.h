/* Copyright 2016 ks */

#ifndef PFF_NODE_PATH_FOLLOWER_H_
#define PFF_NODE_PATH_FOLLOWER_H_

#include <angles/angles.h>
#include <tf/tf.h>
#include "ros/ros.h"

#include "pff_node/trajectory_generator.h"

namespace pff
{
class PathFollower
{
public:
  PathFollower(ros::NodeHandle &nh);
  static tf::Vector3 getVelocity(const tf::Vector3 &p1, const tf::Vector3 &p2)
  {
    tf::Vector3 vel(0, 0, 0);
    double angle_to_goal = atan2(p2.y() - p1.y(), p2.x() - p1.x());
    double angle = angles::shortest_angular_distance(p1.z(), angle_to_goal);
    double distance = pff::TrajectoryGenerator::getDistanceBetweenPoints(p1, p2);

    // @TODO(ks): these values are hardcoded, make adjustable
    double distance_gain = 2.0;
    double turn_gain = 3.5;
    double turn_threshold = 0.5;
    tf::Vector3 max_vel(1, 1, 1);
    max_vel *= 0.5;

    vel.setX(distance * distance_gain);
    vel.setZ(angle * turn_gain);
    if (fabs(angle) > turn_threshold)
    {
      vel.setX(0);
    }

    // clamp
    if (vel.x() > max_vel.x())
    {
      vel.setX(max_vel.x());
    }
    if (vel.x() < -max_vel.x())
    {
      vel.setX(-max_vel.x());
    }

    return vel;
  }

private:
  ros::NodeHandle *nh_;
};

PathFollower::PathFollower(ros::NodeHandle &nh) : nh_(&nh)
{
}

}  // namespace pff

#endif  // PFF_NODE_PATH_FOLLOWER_H_
