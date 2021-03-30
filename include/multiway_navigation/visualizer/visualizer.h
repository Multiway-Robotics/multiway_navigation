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

/**
 * @brief visualizer for debug of hybird navigation
 */

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include "base/macros.h"
#include "base/data_structs.h"
#include "base/land_mark.h"


namespace multiway {

class Visualizer;
using VisualizerHdr = std::shared_ptr<Visualizer>;

class Visualizer
{
public:
  Visualizer(const ros::NodeHandle& node_handle, const bool open=false);
  ~Visualizer();

  /**
   * @brief VisualBox Box visual
   * @param points
   */
  void VisualBox(const std::vector<Point2D>& points);

  /**
   * @brief VisualLine
   * @param start
   * @param end
   */
  void VisualLine(const Point2D& start, const Point2D& end);

  /**
   * @brief VisualCycle cycle visual
   * @param center cycle center
   * @param radius cycle radius
   */
  void VisualCycle(const LandMark &center, const double& radius, const std::string& frame_name);

  /**
   * @brief VisualCycle cycle visual
   * @param center cycle center
   * @param radius cycle radius
   */
  void VisualObservedReflector(const std::vector<LandMark> &land_marks, const std::string& frame_name, const double radius = 0.09);

  /**
   * @brief VisualTransedReflector
   * @param land_marks
   * @param frame_name
   * @param radius
   */
  void VisualTransedReflector(const std::vector<LandMark> &land_marks, const std::string& frame_name, const double radius = 0.09);

  /**
   * @brief VisualCycleArray Visulize the cycle array
   * @param center_points Cycle center array
   * @param radius Cycel radius
   * @param frame_name frame name
   */
  void VisualLocalReflector(const std::vector<LandMark> &center_points, const double& radius, const std::string& frame_name);

  /**
   * @brief CisualPointCloud pointcould visual
   * @param points
   */
  void VisualPointCloud(const std::vector<Point2D>& points, const std::string& frame_name);

  // laser scan visual
  void VisualLaserScan(const sensor_msgs::LaserScan& msg) const;

  /**
   * @brief VisualClusters visiualize the reflector clusters
   * @param clusters
   */
  void VisualClusters(const std::list<Cluster>& clusters, const std::string& frame_name);

private:
  std::vector<geometry_msgs::Point32> GenerateCyclePointsRadius(const Point2D& center, const double& radius) const;

private:
  bool open_;
  std::string frame_name_;

  ros::NodeHandle nh_;
  ros::Publisher line_pub_;
  ros::Publisher cycle_pub_;
  ros::Publisher observed_reflector_pub_;
  ros::Publisher transed_reflector_pub_;
  ros::Publisher scan_filtered_pub_;
  ros::Publisher point_cloud_pub_;
};

}
