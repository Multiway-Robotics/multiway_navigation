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


#include "localizer/reflector_recognizer.h"
#include <experimental/filesystem>

namespace multiway {

ReflectorRecognizer::ReflectorRecognizer(const bool &use_imu)
  : angle_increment_(0.0)
  , current_state_(State::kLocalizing)
  , mean_filter_hdr_(std::make_shared<MeanFilter>(1))
  , laser_corrector_hdr_(std::make_shared<LaserCorrector>())
  , use_imu_(use_imu)
{

}

ReflectorRecognizer::~ReflectorRecognizer()
{

}

void ReflectorRecognizer::AddLaserScan(const sensor_msgs::LaserScan::ConstPtr &data)
{
  mean_filter_hdr_->PushScan(*data);
  if(State::kMapping == current_state_ && robot_vel_.IsStopped()){
    mean_filter_hdr_->SetCachedScanNum(30);
  }else{
    mean_filter_hdr_->SetCachedScanNum(1);
  }

  // @todo: 增加畸变处理
  //sensor_msgs::LaserScan corrected_scan = laser_corrector_hdr_->CorrectLaserScan(*msg);
}

void ReflectorRecognizer::AddOdometryData(const nav_msgs::Odometry::ConstPtr &data)
{
  robot_vel_.x = data->twist.twist.linear.x;
  robot_vel_.y = data->twist.twist.linear.y;
  if(!use_imu_){
    robot_vel_.theta = data->twist.twist.angular.z;
  }
}

void ReflectorRecognizer::AddImuData(const sensor_msgs::Imu::ConstPtr &data)
{
  if(use_imu_){
    robot_vel_.theta = data->angular_velocity.z;
  }
}

const std::vector<LandMark> ReflectorRecognizer::GetLocalReflectors()
{
  // 0. get filtered scan data
  sensor_msgs::LaserScan filtered_scan = mean_filter_hdr_->GetFilteredScan();

  // 1. intensity cluster
  BeamCluster(filtered_scan);

  return FindLocalReflectors();
}

bool ReflectorRecognizer::IsBeamValued(const Beam &input)
{
  double min_intensity = 0.0;
  if(input.distance <= 2.5) {
    min_intensity = kIntensityMin;
  } else {
    double dis = input.distance;
    min_intensity = 0.01997875*dis*dis*dis*dis-1.2575523*dis*dis*dis+28.168*dis*dis-281.34*dis+1800;
  }

  if(input.intensity > min_intensity) {
    return true;
  }else{
    return false;
  }
}

bool ReflectorRecognizer::IsClusterValueable(const Cluster &cluster)
{
  // 0. points number check, less 6 we not process it;
  if(cluster.size() <= 6)
  {
    ROS_ERROR("Points number is too little: %d", cluster.size());
    return false;
  }
  // trim the cluster points according to distance and angle
  // 1. compute this cluster's mean distance
  double distance_sum = 0.0;
  for(auto beam : cluster)
  {
    distance_sum += beam.distance;
  }
  double mean_distance = distance_sum / cluster.size();

  // 2. compute the reasonable points number, according to angle increment and reflctor radius
  int reasonable_points_num = 2 * std::atan((kReflectorRadius + kReflectorRadiusEsp)/mean_distance) / angle_increment_;

  // 3. check if this cluster is reasonable
  if(cluster.size() < std::min(reasonable_points_num, 5))
  {
    ROS_ERROR("Reasonalble points number not match");
    return false;
  }
  //LDROBOT_INFO("Mean distance: %f, Reasonalble points number: %d", mean_distance, reasonable_points_num);
  return true;
}

void ReflectorRecognizer::IntensityCluster(const sensor_msgs::LaserScan &scan)
{
  clusters_.clear();

  angle_increment_ = scan.angle_increment;

  // 1. find all valued clusters
  std::vector<Cluster> raw_clusters;
  Cluster cluster;
  int last_valued_index = 0;
  for(int index = 0; index < scan.ranges.size(); index++)
  {
    Beam beam(scan.ranges.at(index), scan.angle_min + scan.angle_increment * index, scan.intensities.at(index));
    // @todo: the core funtion is IsBeamValued(), so, we must to optimize it
    if(IsBeamValued(beam))
    {
      cluster.push_back(beam);
      last_valued_index = index;
      if(index - last_valued_index > 1) // this need to test
      {
        raw_clusters.push_back(cluster);
        cluster.clear();
      }
    }
  }

  // 2. trim all the unvauled clusters
  for(auto raw_cluster : raw_clusters)
  {
    if(IsClusterValueable(raw_cluster))
    {
      clusters_.push_back(raw_cluster);
    }
    else
    {
      //ROS_TRACE("Trim a unvalued cluster.");
    }
  }
}

void ReflectorRecognizer::BeamCluster(const sensor_msgs::LaserScan &scan)
{
  clusters_.clear();
  angle_increment_ = scan.angle_increment;

  // 1. find all valued clusters
  std::array<Cluster, 1500> raw_clusters;

  int last_valued_index = 0;
  int cluster_count = 0;

  for(int index = 0; index < scan.ranges.size(); index++){
    Beam beam(scan.ranges.at(index), scan.angle_min + scan.angle_increment * index, scan.intensities.at(index));
    if(beam.IsValued()){
      if(0 == cluster_count){
        last_valued_index = index;
        cluster_count++;
        raw_clusters.at(cluster_count - 1).push_back(beam);
        continue;
      }

      if(index - last_valued_index > 1){
        cluster_count++;
      }

      raw_clusters.at(cluster_count - 1).push_back(beam);
      last_valued_index = index;
    }
  }

  // 2. trim all the unvauled clusters
  for(int i = 0; i < cluster_count; i++)
  {
    if(IsClusterValueable(raw_clusters.at(i)))
    {
      clusters_.push_back(raw_clusters.at(i));
    }
    else
    {
      ROS_TRACE("Trim a unvalued cluster.");
    }
  }

}

const std::vector<LandMark> ReflectorRecognizer::FindLocalReflectors()
{
  std::vector<LandMark> ret;

  UInt32 count = 0;
  for(auto cluster : clusters_)
  {
    LandMark land_mark(++count, cluster, kReflectorRadius);
    ret.push_back(land_mark);
  }

  return ret;
}


}
