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

#include "base/landmark_map.h"
#include "localizer/reflector_recognizer.h"
#include "filter/mean_filter.h"
#include "mapper/landmark_mapper.h"
#include "visualizer/visualizer.h"

namespace multiway {

class LocalizerNode;
using LocalizerNodeHdr = std::shared_ptr<LocalizerNode>;

class LocalizerNode
{
public:
  LocalizerNode();
  ~LocalizerNode();

  void SaveMap(const std::string& file_name);

  void AddAllLocalReflectorToMap();

private:
  void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);

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

private:
  // ros stuff
  tf::TransformListener tf_listenner_;
  ros::Subscriber scan_sub_;
  ros::Subscriber signal_add_reflector_sub_;
  ros::Subscriber signal_save_map_sub_;
  std::string laser_frame_name_;

  // logic stuff
  bool is_first_add_;

  // core
  MeanFilterHdr mean_filter_hdr_;                    // scan filter
  ReflectorRecognizerHdr reflector_recognizer_hdr_;  // reflector recognizer
  LandMarkMapHdr landmark_map_hdr_;                  // land mark map
  std::vector<LandMark> local_reflectors_;

  // visulizer
  VisualizerHdr visualizer_hdr_;
};

}
