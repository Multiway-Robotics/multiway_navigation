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

#include <eigen3/Eigen/Dense>
#include <vector>
#include "base/macros.h"

constexpr float kDistanceBia = 0.01;
constexpr float kAngularBis  = 10.0;
constexpr float kIntensityMin = 0.01997875*2.5*2.5*2.5*2.5-1.2575523*2.5*2.5*2.5+28.168*2.5*2.5-281.34*2.5+2000 - 200; // -350
constexpr float kIntensityMax = 1800.0;
constexpr float kDiameterMin = 0.03;
constexpr float kDiameterMax = 0.11;
constexpr float kScanMax = 15.0;

namespace multiway {

using Point2D = Eigen::Vector2d;
using Point3D = Eigen::Vector3d;
using Pose2D = Eigen::Vector3d;

struct Beam{
  Beam(double dis, double ang, double inten)
    : distance(dis)
    , angle(ang)
    , intensity(inten)
  {}

  bool IsValued() const {
    double min_intensity = 0.0;
    if(distance <= 2.5) {
      min_intensity = kIntensityMin;
    }else {
      double dis = distance;
      min_intensity = 0.01997875*dis*dis*dis*dis-1.2575523*dis*dis*dis+28.168*dis*dis-281.34*dis+1800;
    }

    if(intensity > min_intensity) {
      return true;
    }else{
      return false;
    }
  }

  double distance;
  double angle;
  double intensity;
};

struct RobotVel
{
  RobotVel(){ SetZero(); }
  inline void SetZero() { x = y = theta = 0.0; }
  inline bool IsStopped() {
    return (std::fabs(x) < 1e-3 && std::fabs(y) < 1e-3 && std::fabs(theta) < 1e-2);
  }

  double x;
  double y;
  double theta;
};

using Cluster = std::vector<Beam>;
}
