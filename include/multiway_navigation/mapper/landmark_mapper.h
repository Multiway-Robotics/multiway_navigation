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

#include <map>
#include <boost/serialization/map.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include "base/land_mark.h"
#include "localizer/reflector_recognizer.h"
#include "filter/mean_filter.h"

#define REFLECT_DISTANCE 1.0

const std::string kLandMarkMapPath = std::string(getenv("HOME")) + "/.multiway/project/map/landmark_map.xml";

namespace multiway {

class LandMarkMapper;
using LandMarkMapperPtr = LandMarkMapper*;
using LandMarkMapperHdr = boost::shared_ptr<LandMarkMapper>;

class LandMarkMapper
{
public:
  LandMarkMapper();
  ~LandMarkMapper();

  void RunMain();

private:
  void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
  void LaserCallBack_test(const sensor_msgs::LaserScan::ConstPtr& scan);
  void SaveMap(const std_msgs::Bool::ConstPtr &status);
  bool CheckLandmark();
  /**
   * @brief VisualLandMarkMap visual
   */
  void VisualLandMarkMap();

};

} // end of namespace multiway
