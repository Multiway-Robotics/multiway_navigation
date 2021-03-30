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

#include "localizer/landmark_localizer.h"

namespace multiway {

LandMarkLocalizer::LandMarkLocalizer(const bool &use_imu, const std::string &map_file_name)
  : reflector_recognizer_hdr_(std::make_shared<ReflectorRecognizer>(use_imu))
  , landmark_map_hdr_(std::make_shared<LandMarkMap>())
  , least_square_solver_(std::make_shared<LeastSquareSolver>())
  , map_file_name_(map_file_name)
{
  // 1. load the reflector map
  if(!LoadMap(map_file_name_))
  {
    ROS_ERROR("Load reflector map %s failed!", map_file_name_.c_str());
  }
}

LandMarkLocalizer::~LandMarkLocalizer()
{

}

void LandMarkLocalizer::AddLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan, const Pose2D &laser_in_map_pose)
{
  // 1.识别器添加激光数据
  reflector_recognizer_hdr_->AddLaserScan(scan);
  laser_in_map_pose_ = laser_in_map_pose;
}

void LandMarkLocalizer::AddOdometryData(const nav_msgs::Odometry::ConstPtr &data)
{
  // 识别器添加 odometry 数据
  reflector_recognizer_hdr_->AddOdometryData(data);
}

void LandMarkLocalizer::AddImuData(const sensor_msgs::Imu::ConstPtr &data)
{
  // 识别器添加 IMU 数据
  reflector_recognizer_hdr_->AddImuData(data);
}

const bool LandMarkLocalizer::SaveMap(const std::string &file_name)
{
  // write to xml file
  std::ofstream ofs(file_name);
  if(!ofs.good())
  {
    ROS_INFO("无法打开文件: %s, 保存地图失败.", file_name);
    return false;
  }
  boost::archive::xml_oarchive oa(ofs);
  oa << BOOST_SERIALIZATION_NVP(landmark_map_hdr_);
  ofs.close();
  ROS_INFO("保存反光板地图成功.");
  return true;
}

const bool LandMarkLocalizer::LoadMap(const std::string &file_name)
{
  // read from xml file
  std::ifstream ifs(file_name);
  if(!ifs.good())
  {
    ROS_ERROR("landmark map file %s is not exits", file_name);
    return false;
  }

  boost::archive::xml_iarchive ia(ifs);
  ia >> BOOST_SERIALIZATION_NVP(landmark_map_hdr_);
  ifs.close();
  ROS_INFO("Load landmark map success");
  return true;
}

const uint32_t LandMarkLocalizer::AddAllLocalReflectorToMap()
{
  uint insert_number = landmark_map_hdr_->AutoIntertIterms(observed_reflectors_);
  ROS_INFO("Insert %d reflectors to reflector map.", insert_number);
  return insert_number;
}

const Result LandMarkLocalizer::GetResult(Pose2D& pose)
{
  // 1. 识别反光板在激光雷达坐标系下的坐标
  std::vector<LandMark> local_reflectors = reflector_recognizer_hdr_->GetLocalReflectors();

  if(local_reflectors.size() <= 0)
  {
    ROS_ERROR("看到的反光板数量太少.");
    return Result::kLandMarkNumLittle;
  }

  // 2. 将激光雷达坐标系下的反光板，转到地图坐标系
  TransLocalToGlobal(local_reflectors);

  // 2.1 更新当前观测
  observed_reflectors_ = local_reflectors;

  // 3. 求解机器人位姿
  if(landmark_map_hdr_->GetRelectorNum() >= 3)
  {
    // 3.1 遍历地图，搜索匹配的反光板(需要优化算法，加速匹配) <local_id, global_id>
    std::vector<std::pair<LocalLandMark, GlobalLandMark> > matched_reflectors_ = FindMatchedReflectors(local_reflectors);

    // 3.2 如果匹配的反光板数量无法求解，报错
    if(matched_reflectors_.size() < 3) {
      LOG_ERROR("匹配到的反光板数量太少: %d，无法定位.", matched_reflectors_.size());
      return Result::kTooFewMatch;
    }

    // 3.3 根据匹配上的反光板计算车体位姿
    pose = Solve(matched_reflectors_);

    // 3.4 有效性检查
    if(!std::isnan(pose[0]) || !std::isnan(pose[1]) || !std::isnan(pose[2])){
      Pose2D delta_pose = pose - laser_in_map_pose_;
      static int counter = 0;
      if(counter >=1 && (std::fabs(delta_pose[0]) >= 0.15 || std::fabs(delta_pose[1]) >= 0.15)){
        ROS_ERROR("跳变！");
        return Result::kPoseJump;
      }
      counter++;
      return Result::kSolveSuccess;
    }else{
      ROS_ERROR("求解失败");
      return Result::kSolveFailed;
    }
  }else
  {
    ROS_ERROR("空地图.");
    return Result::kEmptyMap;
  }
}

const std::vector<LandMark> LandMarkLocalizer::GetLocalReflectors() const
{
  return observed_reflectors_;
}

const std::vector<std::pair<LocalLandMark, GlobalLandMark> > LandMarkLocalizer::FindMatchedReflectors(std::vector<LandMark> &local_reflectors) const
{
  // 排序，将 local reflector 按照距离排序, 由近到远，取距离最近的
  std::sort(local_reflectors.begin(), local_reflectors.end());

  std::vector<std::pair<LocalLandMark, GlobalLandMark> > ret;
  for(auto iter = local_reflectors.begin(); iter != local_reflectors.end(); iter++){
    int matched_id = landmark_map_hdr_->FindVeryNearItem(*iter);
    if(matched_id >= 0){
      ret.push_back(std::pair<LocalLandMark, GlobalLandMark>(*iter, landmark_map_hdr_->GetLandMarkByID(matched_id)));
    }
    if(ret.size() >= 4)
      break;
  }

  return ret;
}

const bool LandMarkLocalizer::TransLocalToGlobal(std::vector<LandMark> &local_reflectors)
{
  // 2. trans the local to global
  for(auto iter = local_reflectors.begin(); iter != local_reflectors.end(); iter++) {
    iter->UpdateGlobalPos(Pose2Trans(laser_in_map_pose_));
  }
  return true;
}

const Pose2D LandMarkLocalizer::Solve(const std::vector<std::pair<LocalLandMark, GlobalLandMark> > &matched_reflctor)
{

  //auto result_distance = result.second;
  auto result_distance = X0;

  // 1. 求解机器人位姿
  // 1.1 准备数据 <observe, ref>
  uint count = 1;
  std::vector<std::pair<Point3D, Point3D> > pair_points;
  for(auto iter = matched_reflctor.begin(); iter != matched_reflctor.end(); iter++)
  {
    LandMark observe_landmark = iter->first;
    double angle = observe_landmark.GetCenterAngle();
    double x = result_distance(count) * cos(angle);
    double y = result_distance(count) * sin(angle);
    Point3D observe_point(x, y, 1.0);
    count++;

    Point3D ref_point = iter->second.GlobalPoint();

    pair_points.push_back(std::pair<Point3D, Point3D>(observe_point, ref_point));
  }

  // 1.2 求解
  least_square_solver_->SetInput(pair_points, laser_in_map_pose_);
  Pose2D ret = least_square_solver_->ComputeResult();

  return ret;

}

const Eigen::Matrix3d LandMarkLocalizer::Pose2Trans(const Pose2D &pose)
{
  Eigen::Matrix3d ret;
  ret << cos(pose[2]), -sin(pose[2]), pose[0],
         sin(pose[2]),  cos(pose[2]), pose[1],
                    0,             0,       1;

  return ret;
}

const Pose2D LandMarkLocalizer::Trans2Pose(const Eigen::Matrix3d &trans)
{
  Pose2D ret;
  ret[0] = trans(0, 2);
  ret[1] = trans(1, 2);
  ret[2] = atan2(trans(1, 0), trans(0, 0));
  return ret;
}

}


