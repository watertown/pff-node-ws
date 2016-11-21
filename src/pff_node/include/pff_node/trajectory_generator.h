/* Copyright 2016 ks */

#ifndef PFF_NODE_TRAJECTORY_GEN_H_
#define PFF_NODE_TRAJECTORY_GEN_H_

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
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(ros::NodeHandle &nh);
  void setPoints(const std::vector<tf::Vector3> &points);
  void publishGoalMarkers();
  tf::Vector3 getCurrentPoint();
  void setClosestPoint(const tf::Vector3 &p);
  static double getDistanceBetweenPoints(tf::Vector3 p1, tf::Vector3 p2);

  void checkComplete(const tf::Vector3 &p1)
  {
    if (points_.size() == 0)
    {
      return;
    }
    tf::Vector3 p2 = getCurrentPoint();
    double distance_from_goal = getDistanceBetweenPoints(p1, p2);
    if (distance_from_goal < distance_threshold_)
    {
      completePoint();
    }
  }

  void completePoint()
  {
    // ROS_WARN("completed point [%u]", current_index_);
    if (current_index_ == points_.size() - 1)
    {
      current_index_ = 0;
    }
    else
    {
      current_index_++;
    }
  }

  static void createSegmentsFromPoints(int num_segments, double radius, std::vector<tf::Vector3> *points)
  {
    points->resize(0);
    double angle_increment = 2 * M_PI / static_cast<double>(num_segments);
    double s = radius * (1 - cos(angle_increment / 2));
    double rad_prime = radius - s;
    for (int i = 0; i < num_segments; i++)
    {
      double angle = angle_increment * static_cast<double>(i);
      double x = rad_prime * cos(angle);
      double y = rad_prime * sin(angle);
      points->push_back(tf::Vector3(x, y, 0));
    }
    // add the first point again to wrap around
    points->push_back(points->at(0));
  }

  bool setGoalDistanceThreshold(double val)
  {
    if (val < 0)
    {
      return false;
    }
    distance_threshold_ = val;
    return true;
  }

private:
  std_msgs::ColorRGBA makeColor(float r, float g, float b, float a)
  {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  }
  ros::NodeHandle *nh_;
  ros::Publisher marker_pub_;
  std::vector<tf::Vector3> points_;
  uint32_t current_index_;
  tf::Vector3 *current_point_;
  double distance_threshold_;
};

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle &nh)
  : nh_(&nh), current_index_(0), current_point_(NULL), distance_threshold_(0.1)
{
  marker_pub_ = nh_->advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void TrajectoryGenerator::setPoints(const std::vector<tf::Vector3> &points)
{
  points_ = points;
  current_index_ = 0;
}

tf::Vector3 TrajectoryGenerator::getCurrentPoint()
{
  if (current_index_ > points_.size() - 1)
  {
    return tf::Vector3(0, 0, 0);
  }
  return points_[current_index_];
}

double TrajectoryGenerator::getDistanceBetweenPoints(tf::Vector3 p1, tf::Vector3 p2)
{
  // using z as robot angle in 2D, so zero out when computing distance
  p1.setZ(0);
  p2.setZ(0);
  return p1.distance(p2);
}

void TrajectoryGenerator::setClosestPoint(const tf::Vector3 &p)
{
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < points_.size(); i++)
  {
    double dist = getDistanceBetweenPoints(points_[i], p);
    if (dist < min_dist)
    {
      min_dist = dist;
      current_index_ = i;
    }
  }
}

void TrajectoryGenerator::publishGoalMarkers()
{
  if (points_.size() == 0)
  {
    return;
  }
  double alpha = 0.5;
  double lifetime_sec = 0.1;
  std_msgs::ColorRGBA color_point_ = makeColor(0.5, 0.5, 0.5, alpha);
  std_msgs::ColorRGBA color_line_ = makeColor(0.5, 0.5, 0.5, alpha);
  std_msgs::ColorRGBA color_segment_ = makeColor(0.1, 0.1, 0.1, alpha);
  std_msgs::ColorRGBA color_highlight_ = makeColor(0.5, 0.5, 1, 1);

  visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "path";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  points.scale.x = points.scale.y = 0.08;
  line_strip.scale.x = 0.02;
  line_list.scale.x = 0.02;

  // set colors
  points.color = color_point_;
  line_strip.color = color_line_;
  line_list.color = color_segment_;

  points.lifetime = ros::Duration(lifetime_sec);
  line_strip.lifetime = ros::Duration(lifetime_sec);
  line_list.lifetime = ros::Duration(lifetime_sec);

  // Create the vertices for the points and lines
  for (size_t i = 0; i < points_.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = points_[i].x();
    p.y = points_[i].y();
    p.z = points_[i].z();

    if (i == current_index_)
    {
      points.colors.push_back(color_highlight_);
      line_strip.colors.push_back(color_highlight_);
      line_list.colors.push_back(color_highlight_);
    }
    else
    {
      line_strip.colors.push_back(line_strip.color);
      line_list.colors.push_back(line_list.color);
      points.colors.push_back(points.color);
    }

    points.points.push_back(p);
    line_strip.points.push_back(p);

    // The line list needs two points for each line
    line_list.points.push_back(p);
    double height = 0.05;
    p.z += height;
    line_list.points.push_back(p);
  }

  marker_pub_.publish(points);
  marker_pub_.publish(line_strip);
  marker_pub_.publish(line_list);
}

}  // namespace pff

#endif  // PFF_NODE_TRAJECTORY_GEN_H_
