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
 * @brief 把主要的解算代码封装到这个类里面
 */

#pragma once

#include "localizer/reflector_recognizer.h"
#include "base/landmark_map.h"
#include "solver/least_square_solver.h"

namespace multiway {

enum class Result{
  kLandMarkNumLittle = 1,
  kEmptyMap,
  kSolveFailed,
  kSolveSuccess,
  kTooFewMatch,
  kPoseJump,
};

class LandMarkLocalizer;
using LandMarkLocalizerPtr = LandMarkLocalizer*;
using LandMarkLocalizerCPtr = const LandMarkLocalizer*;
using LandMarkLocalizerHdr = std::shared_ptr<LandMarkLocalizer>;

class LandMarkLocalizer final
{
public:
  LandMarkLocalizer(bool use_imu, std::string map_file_name);
  ~LandMarkLocalizer();

  /**
   * @brief AddLaserScan 添加激光数据
   * @param scan
   * @param laser_in_map
   */
  void AddLaserScan(sensor_msgs::LaserScan::ConstPtr scan, const Pose2D &laser_in_map_pose);
  void AddOdometryData(nav_msgs::Odometry::ConstPtr data);
  void AddImuData(sensor_msgs::Imu::ConstPtr data);

  /**
   * @brief SaveMap 保存地图
   * @param file_name
   */
  bool SaveMap(const std::string& file_name);

  bool LoadMap(const std::string &file_name);

  /**
   * @brief SetState 设置状态 构图/定位
   * @param state
   */
  inline void SetState(const State& state) { reflector_recognizer_hdr_->SetState(state); }
  inline State GetState() const { return reflector_recognizer_hdr_->GetState(); }

  uint32_t AddAllLocalReflectorToMap();

  /**
   * @brief GetResult 获取求解结果
   * @param pose
   * @return
   */
  Result GetResult(Pose2D& pose);

  /**
   * @brief GetLocalReflectors 获取激光雷达坐标系下的反光板坐标
   * @return
   */
  std::vector<LandMark> GetLocalReflectors() const;

  inline std::list<Cluster> GetClusters() const { return reflector_recognizer_hdr_->GetClusters(); }
  inline sensor_msgs::LaserScan GetFilteredScan() const { return reflector_recognizer_hdr_->GetFilteredScan(); }

private:
  /**
   * @brief TransLocalToGlobal
   * @param local_reflectors
   * @return
   */
  bool TransLocalToGlobal(std::vector<LandMark> &local_reflectors);

  /**
   * @brief FindMatchedReflectors 与地图信息进行匹配
   * @param local_reflectors
   * @return
   */
  std::vector<std::pair<LocalLandMark, GlobalLandMark> > FindMatchedReflectors(std::vector<LandMark> &local_reflectors) const;

  /**
   * @brief SolveDistance
   * @return
   * @todo optimize code! 代码有点乱
   */
  const Pose2D Solve(const std::vector<std::pair<LocalLandMark, GlobalLandMark> >& matched_reflctor);

  /**
   * @brief Pose2Trans 
   * @param pose
   * @return
   */
  Eigen::Matrix3d Pose2Trans(Pose2D pose);
  Pose2D Trans2Pose(Eigen::Matrix3d trans);

private:
  ReflectorRecognizerHdr reflector_recognizer_hdr_;  // reflector recognizer
  LandMarkMapHdr landmark_map_hdr_;                  // land mark map
  LeastSquareSolverHdr least_square_solver_;
  std::string map_file_name_;

  std::vector<LandMark> observed_reflectors_;
  Pose2D laser_in_map_pose_;
};

}
