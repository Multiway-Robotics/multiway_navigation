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

#include "data_structs.h"
#include <Eigen/Dense>

namespace multiway {

Eigen::Matrix3d Pose2Trans(const Pose2D& pose)
{
  Eigen::Matrix3d ret;
  ret << cos(pose[2]), -sin(pose[2]), pose[0],
         sin(pose[2]),  cos(pose[2]), pose[1],
                    0,             0,       1;

  return ret;
}

const Pose2D Trans2Pose(const Eigen::Matrix3d &trans)
{
  Pose2D ret;
  ret[0] = trans(0, 2);
  ret[1] = trans(1, 2);
  ret[2] = atan2(trans(1, 0), trans(0, 0));
  return ret;
}


}
