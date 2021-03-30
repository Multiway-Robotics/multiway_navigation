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
 * @brief
 */

#pragma once

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "base/landmark_map.h"
#include "localizer/landmark_localizer.h"
#include "filter/mean_filter.h"
#include "filter/laser_corrector.h"
#include "mapper/landmark_mapper.h"
#include "visualizer/visualizer.h"
#include "solver/least_square_solver.h"

namespace multiway {

class MapperNode;
using MapperNodeHdr = std::shared_ptr<MapperNode>;

class MapperNode
{
public:
  MapperNode();
  ~MapperNode();

  void SaveMap(std::string file_name);

  void AddAllLocalReflectorToMap();

private:
  void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);

  void OdomCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  void ImuCallBack(const sensor_msgs::Imu::ConstPtr &msg);

  /**
   * @brief CallBackAddLocalReflector call back funtion of add local reflector to map
   * @param msg
   */
  void CallBackAddLocalReflector(std_msgs::Bool msg);

  /**
   * @brief CallBackSaveMap Call back funtion of save map signal
   * @param msg
   */
  void CallBackSaveMap(std_msgs::Bool msg);

  /**
   * @brief TransLocalToGlobal
   * @param local_reflectors
   * @return
   */
  bool _rt TransLocalToGlobal(std::vector<LandMark> &local_reflectors);

  /**
   * @brief FindMatchedReflectors
   * @param local_reflectors
   * @return <local_id, global_id>
   */
  const std::vector<std::pair<LocalLandMark, GlobalLandMark> > FindMatchedReflectors(std::vector<LandMark> &local_reflectors) const;

  /**
   * @brief SolveDistance
   * @return
   * @todo optimize code! 代码写乱了
   */
  const Pose2D Solve(const std::vector<std::pair<LocalLandMark, GlobalLandMark> >& matched_reflctor) const;

  /**
   * @brief LoadMap Load the reflector map
   * @param file_name map file name
   * @return true if load success, false other wise
   */
  bool LoadMap(std::string file_name);

  void PublishResult(Pose2D data);

  bool LookupTF(const tf::TransformListener &tf_listener, const std::string &target_frame,
                                              const std::string &source_frame, const ros::Time &time,
                                              tf::StampedTransform &transform) const;

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

  std::string laser_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string global_frame_;

  Eigen::Matrix3d laser_in_robot_;

  // core
  ReflectorRecognizerHdr reflector_recognizer_hdr_;  // reflector recognizer
  LandMarkMapHdr landmark_map_hdr_;                  // land mark map
  LeastSquareSolverHdr least_square_solver_;
  std::map<uint, LandMark> local_reflectors_;
  std::vector<LandMark> observed_reflectors_;

  // localization result
  Pose2D laser_pose_;
  tf::Transform odom_in_map_;

  // visulizer
  VisualizerHdr visualizer_hdr_;
};

}
