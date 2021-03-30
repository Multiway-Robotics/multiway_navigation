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

#pragma once

#include "base/landmark_map.h"
#include "localizer/landmark_localizer.h"
#include "ros_node/tf_bridge.h"
#include "visualizer/visualizer.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace multiway {

class RosInterface
{
public:
  RosInterface(bool use_imu, tf2_ros::Buffer * const tf_buffer);
  ~RosInterface() = default;

  void RunMain();

private:
  void CallBackLaserScan(sensor_msgs::LaserScan::ConstPtr msg);
  void CallBackOdometry(nav_msgs::Odometry::ConstPtr msg);
  void CallBackImu(sensor_msgs::Imu::ConstPtr msg);

  void PublishResult(Pose2D data);
  void SendTransform(Pose2D data);
  /**
   * @brief CallBackMapping 开始构图
   * @param msg
   */
  void CallBackMapping(std_msgs::Bool msg);

  /**
   * @brief CallBackMappingEnd 结束构图
   * @param msg
   */
  void CallBackMappingEnd(std_msgs::Bool msg);

  /**
   * @brief CallBackSaveMap 保存地图
   * @param msg
   */
  void CallBackSaveMap(std_msgs::Bool msg);

  /**
   * @brief CallBackAddLocalReflector 添加观测到的反光板到地图
   * @param msg
   */
  void CallBackAddLocalReflector(std_msgs::Bool msg);

  geometry_msgs::Transform ToGeometryMsgTransform(const Pose2D& pose) const;


  const bool LookupTF(const std::string &target_frame, const std::string &source_frame,
                            const ros::Time &time, tf::StampedTransform &transform) const;

private:
  // ros stuff
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber signal_add_reflector_sub_;
  ros::Subscriber signal_save_map_sub_;
  ros::Publisher localize_result_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listenner_;
  VisualizerHdr visualizer_hdr_;
  Pose2D laser_pose_;
  tf::Transform odom_in_map_;

  std::string laser_frame_;
  std::string odom_frame_;
  std::string global_frame_;
  std::string base_frame_;

  LandMarkLocalizerHdr landmark_localizer_hdr_;
  const TfBridge tf_bridge_;
};

}
