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

#include "solver/least_square_solver.h"

namespace multiway {

const Pose2D LeastSquareSolver::ComputeResult()
{
  // 1. construct the optimize problem
  double abc[3] = {init_value_[0], init_value_[1], init_value_[2]};

  ceres::Problem problem;
  for(auto item : pair_points_)
  {
    problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<CorrespondenceResidual, 1, 3>(
            new CorrespondenceResidual(item.first, item.second)), new ceres::CauchyLoss(0.5), abc);
  }

  // 2. setup the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = 2; 
  options.function_tolerance = 2e-3;
  //options.use_nonmonotonic_steps = true; 
  ceres::Solver::Summary summary;

  // 3. solve
  ceres::Solve(options, &problem, &summary);

  // 4. result check
  if(summary.termination_type == ceres::CONVERGENCE)
  {
    return Pose2D(abc[0], abc[1], abc[2]);
  }


  double nan = std::numeric_limits<double>::quiet_NaN();
  return Pose2D(nan, nan, nan);
}

const Eigen::Matrix3d LeastSquareSolver::Pose2Trans(const Pose2D &pose) const
{
  Eigen::Matrix3d ret;
  ret << cos(pose[2]), -sin(pose[2]), pose[0],
         sin(pose[2]),  cos(pose[2]), pose[1],
                    0,             0,       1;

  return ret;
}
}
