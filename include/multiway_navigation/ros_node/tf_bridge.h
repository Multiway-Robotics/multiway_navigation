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

#include <memory>
#include <tf2_ros/buffer.h>
#include <sirius/core/defines.h>
#include <Eigen/Dense>

namespace multiway {

class TfBridge{
public:
    TfBridge(std::string tracking_frame, double lookup_transform_timeout_sec,
             const tf2_ros::Buffer* buffer);
    ~TfBridge() {}

    TfBridge(const TfBridge& other) = delete;
    TfBridge& operator=(const TfBridge& other) = delete;

    std::shared_ptr<Eigen::Matrix3d> LookupToTracking(ros::Time time, std::string frame_id) const;

    std::shared_ptr<Eigen::Matrix3d> LookupToTracking(ros::Time time, std::string tracking_frame_id, std::string frame_id) const;

private:
    Eigen::Matrix3d ToMatrix3d(geometry_msgs::TransformStamped transform) const;

private:
    const std::string tracking_frame_;
    const double lookup_transform_timeout_sec_;
    const tf2_ros::Buffer* const buffer_;
};

}
