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
 * @brief MeanFilter
 * @todo consider the robot motion
 */

#pragma once

#include <list>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <inttypes.h>
#include "base/data_structs.h"

constexpr float kNoiseThreshold = 0.07;

namespace multiway {

struct LandScan
{
  double robot_angle;
  sensor_msgs::LaserScan scan;
};

class MeanFilter;
using MeanFilterHdr = std::shared_ptr<MeanFilter>;

class MeanFilter{
public:
  MeanFilter(const int cache_scans_nums = 5, const bool use_odom_compensate = false);
  ~MeanFilter();

  /**
   * @brief SetCachedScanNum used to set change the filter param dynamicly
   * @param scan_num
   */
  inline void SetCachedScanNum(const int& scan_num){
    if(scan_num > 0 && scan_num < 50)
      max_cache_scans_nums_ = scan_num;
  }

  /**
   * @brief SetFilterParamFromPose dynamicly change the filter parameter
   * @param pose
   */
  void SetFilterParamFromPose(const Pose2D& pose);

  /**
   * @brief PushScan push the raw laser scan to the filter
   * @param scan laser scan message
   */
  void PushScan(const sensor_msgs::LaserScan& scan);

  void PushScanAdvanced(const sensor_msgs::LaserScan& scan, const nav_msgs::Odometry& odom);

  /**
   * @brief ClearScan clear the history cached scans
   */
  inline void ClearScan(){
    cached_scans_.clear();
  }

  /**
   * @brief GetFilteredScan
   * @return filtered laser scan
   */
  const sensor_msgs::LaserScan GetFilteredScan();

  /**
   * @brief GetFilteredScanNew I try to combine the odom to remove the laser distortion, but failed
   * @todo
   */
  const sensor_msgs::LaserScan GetFilteredScanAdvanced();

  /**
   * @brief GetCachedScanNum get the cached scan number
   * @return cached scan number
   */
  inline const int GetCachedScanNum() const {
    return max_cache_scans_nums_;
  }

private:
  void NoiseFilter();

private:
  bool use_odom_compensate_;
  bool is_scan_init_;
  int max_cache_scans_nums_;
  std::list<sensor_msgs::LaserScan> cached_scans_;
  std::list<LandScan> cached_landscans_;
  sensor_msgs::LaserScan scan_filtered_;
}; // end of class MeanFilter

}  // end of namespace ldrobot
