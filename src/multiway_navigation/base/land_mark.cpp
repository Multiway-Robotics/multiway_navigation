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

#include "base/land_mark.h"

namespace multiway {

LandMark::LandMark(const uint32_t &id, const Cluster &cluster, const double &radius)
  : local_id_(id)
  , beams_(cluster)
  , radius_(radius)
{
  Init();
}

void LandMark::Init()
{
  // 1. compute angle center
  double angle_sum = 0;
  for(auto beam : beams_)
  {
    angle_sum += beam.angle;
  }
  double angle_center = angle_sum/beams_.size();

  // 2. compute distance
  double radius_sum = 0;
  for(auto beam : beams_)
  {
    double delta_theta = std::fabs(beam.angle - angle_center);
    //radius_sum += (beam.distance * cos(delta_theta) + kReflectorRadius * cos(M_PI/2 - delta_theta));
    double rl = beam.distance * sin(delta_theta);
    //LDROBOT_TRACE("RL: %f", rl);
    if(rl >= kReflectorRadius)
    {
      radius_sum += beam.distance * cos(delta_theta);
      //LDROBOT_WARNING("unvalued distance value");
    }else
    {
      radius_sum += (beam.distance * cos(delta_theta) + sqrt(kReflectorRadius*kReflectorRadius -  rl * rl));
    }
  }
  double radius = radius_sum / beams_.size();

  cluster_center_ = angle_center;
  local_x_ = radius * cos(angle_center);
  local_y_ = radius * sin(angle_center);
}

void LandMark::UpdateGlobalPos(const Eigen::Matrix3d &local_in_global)
{
  Point3D global_point = local_in_global * LocalPoint();
  global_x_ = global_point[0];
  global_y_ = global_point[1];
}



}
