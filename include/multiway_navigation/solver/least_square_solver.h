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
 * @brief Core module defines
 */

#pragma once

#include <ceres/ceres.h>
#include "base/data_structs.h"

namespace multiway {

// cost function: reflector observed to map reflector
struct CorrespondenceResidual
{
  CorrespondenceResidual(const Point3D& point, const Point3D& ref_point)
    : _point(point)
    , _ref_point(ref_point)
  {}

  template<typename T>
  bool operator()(const T* const abc, T* residual) const
  {
    T x = cos(abc[2]) * _point[0] - sin(abc[2]) * _point[1] + abc[0];
    T y = sin(abc[2]) * _point[0] + cos(abc[2]) * _point[1] + abc[1];

    // effect, can find the global minima
    residual[0] = sqrt((_ref_point[0] - x)*(_ref_point[0] - x) + (_ref_point[1] - y)*(_ref_point[1] - y));
    return true;
  }

private:
  const Point3D _point;
  const Point3D _ref_point;
};


class LeastSquareSolver;
using LeastSquareSolverHdr = std::shared_ptr<LeastSquareSolver>;

class LeastSquareSolver final
{
public:
  LeastSquareSolver() = default;
  ~LeastSquareSolver() = default;

  /**
   * @brief SetInput
   * @param pair_points <local_point, global_point>
   * @param init_value you know, we need a initial value
   */
  inline void SetInput(const std::vector<std::pair<Point3D, Point3D> >& pair_points, const Pose2D& init_value)
  {
    pair_points_ = pair_points;
    init_value_ = init_value;
  }

  Pose2D ComputeResult();

private:
  inline Eigen::Matrix3d Pose2Trans(Pose2D pose) const;
  inline Pose2D Trans2Pose(Eigen::Matrix3d trans) const;
  inline double NormalizeAngle(double angle) const;
  inline Pose2D ResultCheck(std::vector<Pose2D> rets) const;

  /**
   * @brief ResultCost compute the result's cost used to check the result isn't useable
   * @param pose triangle pose in laser frame
   * @return cost
   */
  inline double ResultCost(Pose2D pose) const;

private:
  std::vector<std::pair<Point3D, Point3D> > pair_points_;
  Pose2D init_value_;
};

}
