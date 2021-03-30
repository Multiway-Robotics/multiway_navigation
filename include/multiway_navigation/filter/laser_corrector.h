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
 * @brief 匀速模型
 * @todo: 采用更加精确的插值模型做
 */

#pragma once

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tuple>

namespace multiway {

class LaserCorrector;
using LaserCorrectorHdr = std::shared_ptr<LaserCorrector>;

class LaserCorrector final{
public:
  struct RobotVel{
    double vx;
    double vy;
    double vw;
  };
  LaserCorrector(const uint cached_number = 10);
  ~LaserCorrector() = default;

  void AddOdometryData(const nav_msgs::Odometry& data);
  void AddImuData(const sensor_msgs::Imu& data);

  const sensor_msgs::LaserScan CorrectLaserScan(const sensor_msgs::LaserScan& scan);

private:
  RobotVel GetRobotVel() const;

private:
  nav_msgs::Odometry current_odom_;
  Eigen::Vector3f velocity_;
  std::list<double> vx_list_;
  std::list<double> imu_angles_;
  uint cached_number_;
};

}

