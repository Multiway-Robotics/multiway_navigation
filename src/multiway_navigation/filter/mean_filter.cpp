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


#include "filter/mean_filter.h"
#include <angles/angles.h>
#include <tf/tf.h>
#include <boost/algorithm/clamp.hpp>

namespace multiway {


MeanFilter::MeanFilter(const int cache_scans_nums, const bool use_odom_compensate)
  : use_odom_compensate_(use_odom_compensate)
  , is_scan_init_(false)
  , max_cache_scans_nums_(cache_scans_nums)
{

}

MeanFilter::~MeanFilter()
{

}

void MeanFilter::SetFilterParamFromPose(const Pose2D &pose)
{
  double distance = std::sqrt(pose[0] * pose[0] + pose[1] + pose[1]);
  int number = std::floor(-18.75 * distance + 23.75); // xiang xia qu zheng
  // resctrit to rangge (1 20)
  number = boost::algorithm::clamp(number, 5, 20);
  SetCachedScanNum(number);
}

void MeanFilter::PushScan(const sensor_msgs::LaserScan &scan)
{
  if(!is_scan_init_)
  {
    scan_filtered_.header.frame_id = scan.header.frame_id;
    scan_filtered_.angle_increment = scan.angle_increment;
    scan_filtered_.angle_min = scan.angle_min;
    scan_filtered_.angle_max = scan.angle_max;
    scan_filtered_.range_min = scan.range_min;
    scan_filtered_.range_max = scan.range_max;
    scan_filtered_.ranges.resize(scan.ranges.size());
    scan_filtered_.intensities.resize(scan.intensities.size());

    is_scan_init_ = true;
  }
  cached_scans_.push_back(scan);
  while (cached_scans_.size() > max_cache_scans_nums_)
  {
    cached_scans_.pop_front();
  }
}

void MeanFilter::PushScanAdvanced(const sensor_msgs::LaserScan &scan, const nav_msgs::Odometry &odom)
{
  double current_angle = tf::getYaw(odom.pose.pose.orientation);

  LandScan land_scan;
  land_scan.robot_angle = current_angle;
  land_scan.scan = scan;

  if(!is_scan_init_)
  {
    scan_filtered_.header.frame_id = scan.header.frame_id;
    scan_filtered_.angle_increment = scan.angle_increment;
    scan_filtered_.angle_min = scan.angle_min;
    scan_filtered_.angle_max = scan.angle_max;
    scan_filtered_.range_min = scan.range_min;
    scan_filtered_.range_max = scan.range_max;
    scan_filtered_.ranges.resize(scan.ranges.size());
    scan_filtered_.intensities.resize(scan.intensities.size());

    is_scan_init_ = true;
  }
  cached_landscans_.push_back(land_scan);

  while (cached_landscans_.size() > max_cache_scans_nums_)
  {
    cached_landscans_.pop_front();
  }
}

const sensor_msgs::LaserScan MeanFilter::GetFilteredScan()
{
  sensor_msgs::LaserScan ret;

  if(cached_scans_.size() < 1)
  {
    return ret;
  }

  // mean filter
  for(int i = 0; i < scan_filtered_.ranges.size(); i++)
  {
    int counter = 0;
    double range = 0.0;
    double intensity = 0.0;
    for(std::list<sensor_msgs::LaserScan>::const_iterator iter = cached_scans_.begin(); iter != cached_scans_.end(); iter ++)
    {
      // @todo: just filterd according to the min and max distance, Is this valuable?
      if(iter->ranges[i] < scan_filtered_.range_max && iter->ranges[i] > scan_filtered_.range_min)
      {
        counter++;
        range += iter->ranges[i];
        intensity += iter->intensities[i];
      }
    }
    range = range / counter;
    intensity = intensity / counter;
    scan_filtered_.ranges[i] = range;
    scan_filtered_.intensities[i] = intensity;
  }

  // noise filter
  // NoiseFilter();

  //scan_filtered_.intensities = cached_scans_.rbegin()->intensities;

  scan_filtered_.header.stamp = ros::Time::now();
  return scan_filtered_;
}

const sensor_msgs::LaserScan MeanFilter::GetFilteredScanAdvanced()
{
  if(cached_landscans_.size() < 1)
  {
    sensor_msgs::LaserScan ret;
    return ret;
  }

  LandScan latest_landscan;
  latest_landscan = cached_landscans_.back();

  std::vector<LandScan> corrected_landscan_list;

  // transform to new frame accoding to robot angle
  for(std::list<LandScan>::const_iterator iter = cached_landscans_.begin(); iter != cached_landscans_.end(); iter ++)
  {
    LandScan corrected_scan;
    corrected_scan.robot_angle = latest_landscan.robot_angle;
    corrected_scan.scan = scan_filtered_;

    double landscan_angle = iter->robot_angle;

    int motion_increment_index = -std::ceil((landscan_angle - latest_landscan.robot_angle)/scan_filtered_.angle_increment);

    ROS_ERROR("motion_increment index = %d", motion_increment_index);

    for(int i = 0; i < corrected_scan.scan.ranges.size(); i++)
    {
      if((i + motion_increment_index) >= corrected_scan.scan.ranges.size())
      {
        break;
      }
      if((i + motion_increment_index) < 0)
      {
        corrected_scan.scan.ranges[i] = 0.0;
      }
      if( (i + motion_increment_index) >= 0)
      {
        corrected_scan.scan.ranges[i] = iter->scan.ranges[i + motion_increment_index];
      }
    }

    corrected_landscan_list.push_back(corrected_scan);
  }

  for(int i = 0; i < scan_filtered_.ranges.size(); i++)
  {
    double range = 0.0;
    for(auto landscan:corrected_landscan_list)
    {
      range += landscan.scan.ranges[i];
    }
    range = range / corrected_landscan_list.size();
    scan_filtered_.ranges[i] = range;
  }

  return scan_filtered_;
}

void MeanFilter::NoiseFilter()
{
  if (scan_filtered_.ranges.size() < 3){
    return;
  }

  int counter = 0;
  unsigned int p_i, p_j, p_k;
  for (std::size_t i = 0; i < scan_filtered_.ranges.size(); ++i) {
    // Get two closest neighbours
    p_i = scan_filtered_.ranges[i];
    if (i == 0) // first point
    {
      p_j = scan_filtered_.ranges[i + 1];
      p_k = scan_filtered_.ranges[i + 2];
    }
    else if (i == scan_filtered_.ranges.size() - 1) // last point
    {
      p_j = scan_filtered_.ranges[i - 1];
      p_k = scan_filtered_.ranges[i - 2];
    }
    else // middle points
    {
      p_j = scan_filtered_.ranges[i - 1];
      p_k = scan_filtered_.ranges[i + 1];
    }

    // Check if point is an outlier
    if (fabs(p_i - p_j) > 1.5*kNoiseThreshold && fabs(p_i - p_k) > 1.5*kNoiseThreshold)  // all >0.053m seen as outlier
    {
      scan_filtered_.ranges[i] = scan_filtered_.range_max;
      counter++;
    }
  }
}

} // end of namespace ldrobot

