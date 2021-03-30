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


#include "ros_node/tf_bridge.h"
#include <tf/tf.h>

namespace multiway {

TfBridge::TfBridge(const std::string &tracking_frame,
                   const double &lookup_transform_timeout_sec,
                   const tf2_ros::Buffer *buffer)
  : tracking_frame_(tracking_frame)
  , lookup_transform_timeout_sec_(lookup_transform_timeout_sec)
  , buffer_(buffer) {}

std::shared_ptr<Eigen::Matrix3d> TfBridge::LookupToTracking(const ros::Time &time, const std::string &frame_id) const
{
  ::ros::Duration timeout(lookup_transform_timeout_sec_);

  try{
    const ::ros::Time latest_tf_time = buffer_->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.), timeout).header.stamp;
    const ::ros::Time requested_time = time;
    if(latest_tf_time >= requested_time){
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is tracking_frame_too old.
      timeout = ::ros::Duration(0.);
    }
    return std::make_shared<Eigen::Matrix3d>(ToMatrix3d(buffer_->lookupTransform(tracking_frame_, frame_id, latest_tf_time, timeout)));
  }
  catch (tf2::TransformException _in ex){
    ROS_WARNING("Tf 查询异常: %s", ex.what());
  }
  return nullptr;
}

std::shared_ptr<Eigen::Matrix3d> TfBridge::LookupToTracking(const ros::Time &time, const std::string &tracking_frame_id, const std::string &frame_id) const
{
  ::ros::Duration timeout(lookup_transform_timeout_sec_);

  try{
    const ::ros::Time latest_tf_time = buffer_->lookupTransform(tracking_frame_id, frame_id, ::ros::Time(0.), timeout).header.stamp;
    const ::ros::Time requested_time = time;
    if(latest_tf_time >= requested_time){
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is tracking_frame_too old.
      timeout = ::ros::Duration(0.);
    }
    return std::make_shared<Eigen::Matrix3d>(ToMatrix3d(buffer_->lookupTransform(tracking_frame_id, frame_id, requested_time, timeout)));
  }
  catch (tf2::TransformException _in ex){
    ROS_WARNING("Tf 查询异常: %s", ex.what());
  }
  return nullptr;
}

Eigen::Matrix3d TfBridge::ToMatrix3d(const geometry_msgs::TransformStamped &transform) const
{
  Eigen::Matrix3d ret;
  double yaw = tf::getYaw(transform.transform.rotation);
  ret << cos(yaw), -sin(yaw), transform.transform.translation.x,
         sin(yaw),  cos(yaw), transform.transform.translation.y,
                    0,             0,       1;

  return ret;

}



}
