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

#include "filter/laser_corrector.h"
#include <boost/make_shared.hpp>
#include <tf/tf.h>

namespace multiway {

LaserCorrector::LaserCorrector(const uint cached_number)
  : cached_number_(cached_number)
{

}

void LaserCorrector::AddOdometryData(const nav_msgs::Odometry &data)
{
  vx_list_.push_back(data.twist.twist.linear.x);
  while(vx_list_.size() > cached_number_){
    vx_list_.pop_front();
  }
}

void LaserCorrector::AddImuData(const sensor_msgs::Imu &data)
{
  imu_angles_.push_back(data.angular_velocity.z);

  while(imu_angles_.size() > cached_number_){
    imu_angles_.pop_front();
  }
}

const sensor_msgs::LaserScan LaserCorrector::CorrectLaserScan(const sensor_msgs::LaserScan &scan)
{
  // 1. 初始化 数据头
  sensor_msgs::LaserScan filtered_scan;
  filtered_scan.header = scan.header;
  filtered_scan.angle_min = scan.angle_min;
  filtered_scan.angle_max = scan.angle_max;
  filtered_scan.angle_increment = scan.angle_increment;
  filtered_scan.range_min = scan.range_min;
  filtered_scan.range_max = scan.range_max;
  filtered_scan.time_increment = scan.time_increment;
  filtered_scan.scan_time = scan.scan_time;
  filtered_scan.ranges.resize(scan.ranges.size());
  filtered_scan.intensities.resize(scan.intensities.size());

  // 2. 开始矫正
  for(size_t i = 0; i < scan.ranges.size(); i++){
    const double range = scan.ranges.at(i);
    if(std::isnan(range) || range < scan.range_min || range > scan.range_max){
      continue;
    }

    RobotVel vel = GetRobotVel();

    double delta_t = scan.time_increment * i;
    float move_angle = delta_t * vel.vw;

    // t1 时刻机器人 在 t0 时刻机器人坐标系下的坐标转换
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(vel.vx*std::cos(move_angle)*delta_t, vel.vx * std::sin(move_angle)*delta_t, 0));
    trans.setRotation(tf::createQuaternionFromYaw(move_angle));

    const double laser_angle = scan.angle_min + scan.angle_increment * i;
    tf::Vector3 laser_point(range * std::cos(laser_angle), range * std::sin(laser_angle), 0 );
    tf::Vector3 correct_point = trans.inverse() * laser_point;

    const double new_range = std::sqrt(correct_point.x() * correct_point.x()
                                       + correct_point.y() * correct_point.y());
    const double new_angle = std::atan2(correct_point.y(), correct_point.x());
    const size_t index = round((new_angle - scan.angle_min)/scan.angle_increment);

    // 填充
    filtered_scan.ranges[index] = new_range;
    filtered_scan.intensities[index] = scan.intensities[index];
  }

  return filtered_scan;
}

const LaserCorrector::RobotVel LaserCorrector::GetRobotVel() const
{
   RobotVel ret;
   double vel_xsum = 0.0;
   size_t count = 0;
   for(auto vx : vx_list_){
     vel_xsum += vx;
     count++;
   }
   ret.vx = vel_xsum / count;

   double vel_wsum = 0.0;
   count = 0;
   for(auto vw : imu_angles_){
     vel_wsum += vw;
     count++;
   }
   ret.vw = vel_wsum / count;

   return ret;
}




}
