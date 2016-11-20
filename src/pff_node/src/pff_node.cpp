/* Copyright 2016 ks */

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "pff_node/constants.h"
#include "pff_node/mobile_robot.h"
#include "pff_node/path_follower.h"
#include "pff_node/trajectory_generator.h"

typedef std::map<std::string, pff::TrajectoryGenerator> trajectory_map_t;
typedef trajectory_map_t::iterator trajectory_iterator_t;
trajectory_map_t trajectories;                 ///< map container to
                                               ///< store trajectories
                                               ///< by mode e.g. "circle"
pff::TrajectoryGenerator *current_trajectory;  ///< pointer to current
                                               ///< trajectory
pff::MobileRobot robot;                        ///< stores odometry information

// ROS parameters with default values
bool autonomous = true;                          ///< if true, robot will
                                                 ///< start following the path
std::string mode(pff::TrajectoryName::kCircle);  ///< mode
double circle_diameter = 1.0;                    ///< diameter of circle path
double square_side_length = 1.0;                 ///< side length of square path

bool selectTrajectoryByName(std::string name)
{
  trajectory_iterator_t it;
  it = trajectories.find(name);
  if (it == trajectories.end())
  {
    ROS_WARN("[%s] Invalid trajectory", name.c_str());
    return false;
  }
  ROS_WARN("[%s] Select trajectory", name.c_str());
  current_trajectory = &it->second;
  return true;
}

void selectModeCallback(const std_msgs::String::ConstPtr &msg)
{
  selectTrajectoryByName(msg->data);
  current_trajectory->setClosestPoint(robot.getPosition());
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  tf::Vector3 new_vel(msg->linear.x, 0, msg->angular.z);
  robot.setVelocity(new_vel);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pff_node");
  ros::NodeHandle nh("~");

  // read ROS parameters
  nh.param<std::string>("mode", mode, mode);
  nh.param<bool>("autonomous", autonomous, autonomous);
  nh.param<double>("circle_diameter", circle_diameter, circle_diameter);
  nh.param<double>("square_side_length", square_side_length, square_side_length);

  robot.init(nh);

  // subscribers
  ros::Subscriber sub_select_mode = nh.subscribe<std_msgs::String>("select_mode", 1, selectModeCallback);
  ros::Subscriber sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelCallback);

  // create trajectories
  pff::TrajectoryGenerator circle_trajectory(nh);
  pff::TrajectoryGenerator square_trajectory(nh);

  // create trajectories
  std::vector<tf::Vector3> points;
  pff::TrajectoryGenerator::createSegmentsFromPoints(30, circle_diameter / 2, &points);
  circle_trajectory.setPoints(points);
  pff::TrajectoryGenerator::createSegmentsFromPoints(4, square_side_length / 2 / (0.707), &points);
  square_trajectory.setPoints(points);

  // add to map
  trajectories.insert(
      std::pair<std::string, pff::TrajectoryGenerator>(pff::TrajectoryName::kCircle, circle_trajectory));
  trajectories.insert(
      std::pair<std::string, pff::TrajectoryGenerator>(pff::TrajectoryName::kSquare, square_trajectory));

  bool success = selectTrajectoryByName(mode);
  if (!success)
  {
    ROS_ERROR("[%s] Invalid mode.", mode.c_str());
    return -1;
  }

  current_trajectory->setGoalDistanceThreshold(0.3);
  current_trajectory->setClosestPoint(robot.getPosition());

  ros::Rate r(30.0);

  while (nh.ok())
  {
    ros::spinOnce();

    current_trajectory->checkComplete(robot.getPosition());

    tf::Vector3 vel = pff::PathFollower::getVelocity(robot.getPosition(), current_trajectory->getCurrentPoint());

    if (autonomous)
    {
      robot.setVelocity(vel);
      current_trajectory->publishGoalMarkers();
    }

    robot.publishTf();

    r.sleep();
  }

  return 0;
}
