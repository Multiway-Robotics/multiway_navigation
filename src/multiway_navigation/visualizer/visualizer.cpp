/*
 * Copyright 2021 The Mutiway Robotics Authors
 *
 * Licensed under the GPL License, Version 3.0 (the "License");
 * you may not use this file except in compliance with the License.

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "visualizer/visualizer.h"
#include <visualization_msgs/MarkerArray.h>

namespace multiway {

Visualizer::Visualizer(const ros::NodeHandle &node_handle, const bool open)
  : nh_(node_handle)
  , open_(open)
{
  cycle_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("visualizer_cycle", 3, true);
  observed_reflector_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualizer_observed_reflector", 3, true);
  transed_reflector_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualizer_transed_reflector", 3, true);
  line_pub_ = nh_.advertise<visualization_msgs::Marker>("visualizer_line", 3, true);
  scan_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>("visualizer_scan_debug", 3, true);
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("visualizer_cluster_points", 2, true);
}

Visualizer::~Visualizer(){}

void Visualizer::VisualLine(const Point2D &start, const Point2D &end)
{
  if(!open_)
  {
    return;
  }

  visualization_msgs::Marker marker_msg;
  marker_msg.ns = "strip_detector";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.01;
  marker_msg.color.r = 0;
  marker_msg.color.g = 1;
  marker_msg.color.b = 0;
  marker_msg.color.a = 1.0;

  geometry_msgs::Point p_a, p_b;
  p_a.x = start[0];
  p_a.y = start[1];
  p_a.z = 0;
  p_b.x = end[0];
  p_b.y = end[1];

  marker_msg.points.push_back(p_a);
  marker_msg.points.push_back(p_b);

  marker_msg.header.frame_id = "laser_scanner_link_top";
  marker_msg.header.stamp = ros::Time::now();// @todo: add it //= ros::Time::now();

  line_pub_.publish(marker_msg);
}

void Visualizer::VisualCycle(const LandMark &center, const double &radius, std::string _in frame_name)
{
  if(open_)
  {
    geometry_msgs::PolygonStamped cycle_msg;
    cycle_msg.header.frame_id = frame_name;
    //cycle_msg.header.stamp = ros::Time::now();
    Point2D center_point(center.GetLocalX(), center.GetLocalY());
    cycle_msg.polygon.points = GenerateCyclePointsRadius(center_point, radius);

    cycle_pub_.publish(cycle_msg);
  }
}

void Visualizer::VisualObservedReflector(const std::vector<LandMark> &land_marks, const std::string &frame_name, const double radius)
{
  if(!open_)
    return;

  visualization_msgs::MarkerArray markers_msg;

  size_t count = 0;
  for(auto iter = land_marks.begin(); iter != land_marks.end(); iter++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "reflector_detector";
    marker.id = count++;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.01;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;

    geometry_msgs::Pose pose;
    pose.position.x = (*iter).GetLocalX(); //land_marks.at(i).GetX();
    pose.position.y = (*iter).GetLocalY();
    pose.position.z = 0.0;
    marker.pose=pose;
    markers_msg.markers.push_back(marker);
  }

  observed_reflector_pub_.publish(markers_msg);
}

void Visualizer::VisualTransedReflector(const std::vector<LandMark> &land_marks, const std::string &frame_name, const double radius)
{
  if(!open_)
    return;

  visualization_msgs::MarkerArray markers_msg;

  size_t count = 0;
  for(auto iter = land_marks.begin(); iter != land_marks.end(); iter++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "reflector_detector";
    marker.id = ++count;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.01;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;

    geometry_msgs::Pose pose;
    pose.position.x = (*iter).GetGlobalX(); //land_marks.at(i).GetX();
    pose.position.y = (*iter).GetGlobalY();
    pose.position.z = 0.0;
    marker.pose=pose;
    markers_msg.markers.push_back(marker);
  }

  transed_reflector_pub_.publish(markers_msg);
}

void Visualizer::VisualLocalReflector(const std::vector<LandMark> &center_points, const double &radius, const std::string &frame_name)
{
  if(!open_)
    return;

  for(int i = 0; i < center_points.size(); i++)
  {
    VisualCycle(center_points.at(i), radius, frame_name);
  }
}

void Visualizer::VisualPointCloud(const std::vector<Point2D> &points, std::string _in frame_name)
{
  if(!open_)
  {
    return;
  }
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = frame_name;
  msg.header.stamp = ros::Time::now();

  geometry_msgs::Point32 point_msg;
  for(auto point : points)
  {
    point_msg.x = point[0];
    point_msg.y = point[1];
    point_msg.z = 0.0;
    msg.points.push_back(point_msg);
  }

  point_cloud_pub_.publish(msg);
}

void Visualizer::VisualLaserScan(const sensor_msgs::LaserScan &msg) const
{
  if(!open_)
  {
    return;
  }
  scan_filtered_pub_.publish(msg);
}

void Visualizer::VisualClusters(const std::list<Cluster> &clusters, const std::string &frame_name)
{
  if(!open_)
    return;

  sensor_msgs::PointCloud msg;
  msg.header.frame_id = frame_name;
  msg.header.stamp = ros::Time::now();

  geometry_msgs::Point32 point_msg;
  for(auto cluster : clusters)
  {
    for(auto beam : cluster)
    {
      point_msg.x = beam.distance * cos(beam.angle);
      point_msg.y = beam.distance * sin(beam.angle);
      point_msg.z = 0.0;
      msg.points.push_back(point_msg);
    }
  }
  point_cloud_pub_.publish(msg);
}

std::vector<geometry_msgs::Point32> Visualizer::GenerateCyclePointsRadius(const Point2D &center, const double &radius) const
{
  std::vector<geometry_msgs::Point32> points;
  // Loop over 64 angles around a circle making a point each time
  int number = 64;
  geometry_msgs::Point32 pt;
  for ( int i = 0; i < number; ++i )
  {
    double angle = i * 2 * M_PI / number;
    pt.x = cos ( angle ) * radius + center[0];
    pt.y = sin ( angle ) * radius + center[1];
    points.push_back ( pt );
  }
  return points;
}

}
