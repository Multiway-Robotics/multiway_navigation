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
 * @brief Reflector recognizer, complete the reflector recognize funtion
 */

#pragma once

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "base/data_structs.h"
#include "base/land_mark.h"
#include "filter/mean_filter.h"
#include "filter/laser_corrector.h"

namespace multiway {

enum class State
{
  kMapping = 0,
  kLocalizing
};

class ReflectorRecognizer;
using ReflectorRecognizerPtr = ReflectorRecognizer*;
using ReflectorRecognizerCPtr = const ReflectorRecognizer*;
using ReflectorRecognizerHdr = std::shared_ptr<ReflectorRecognizer>;

class ReflectorRecognizer {
public:
  ReflectorRecognizer(bool _in use_imu);
  ~ReflectorRecognizer();

  /**
   * @brief PushScan push scan data to recognizer
   * @param scan laser scan data
   */
  void AddLaserScan(sensor_msgs::LaserScan::ConstPtr data);
  void AddOdometryData(nav_msgs::Odometry::ConstPtr data);
  void AddImuData(sensor_msgs::Imu::ConstPtr data);

  /**
   * @brief RecognizeReflectorPositions Recognize the reflector positions in laser frame, used for mapper, PURE implementation
   * @param scan Scan message
   * @return reflector positions in laser frame
   */
  std::vector<LandMark> GetLocalReflectors();

  /**
   * @brief GetClusters get the reflector clusters
   * @return
   */
  inline std::list<Cluster> GetClusters() const { return clusters_; }

  /**
   * @brief GetFilterdScan Get filtered scan
   * @return filtered scan
   */
  inline sensor_msgs::LaserScan GetFilteredScan() const { return mean_filter_hdr_->GetFilteredScan(); }

  /**
   * @brief SetState 设置状态 构图/定位
   * @param state
   */
  inline void SetState(const State& state) { current_state_ = state; }
  inline State GetState() const { return current_state_; }

private:
  /**
   * @brief CheckIntensity check is the input beam is on the reflector
   * @param input input beam
   * @return true if is valued, false othter wise
   */
  bool IsBeamValued(Beam input);

  /**
   * @brief IsClusterValueable check if the cluster is valueable
   * @param cluster
   * @return
   */
  bool IsClusterValueable(const Cluster &cluster);

  /**
   * @brief IntensityCluster cluster points according to intensity
   * @param msg laser message
   */
  void IntensityCluster(const sensor_msgs::LaserScan& scan);

  /**
   * @brief BeamCluster Beam cluster
   * @param scan
   */
  void BeamCluster(const sensor_msgs::LaserScan& scan);

  /**
   * @brief FindLocalReflectors
   * @return
   * @todo Optimize it
   */
  std::vector<LandMark> FindLocalReflectors();

private:
  std::list<Cluster> clusters_;
  double angle_increment_;
  bool use_imu_;

  MeanFilterHdr mean_filter_hdr_;
  LaserCorrectorHdr laser_corrector_hdr_;

  State current_state_;
  RobotVel robot_vel_;
};

}
