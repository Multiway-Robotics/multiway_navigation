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


#include "ros_node/ros_interface.h"
#include "base/math.h"

namespace multiway {

const std::string kMapPath = std::string(getenv("HOME")) + "/.multiway/map/reflector_map.xml";

RosInterface::RosInterface(const bool &use_imu, tf2_ros::Buffer * const tf_buffer)
  : landmark_localizer_hdr_(std::make_shared<LandMarkLocalizer>(use_imu, kMapPath))
  , tf_bridge_("map", 0.2, tf_buffer)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string laser_topic, odom_topic, reflector_pose_topic;
  pnh.param("laser_topic", laser_topic, std::string("/scan_head"));
  pnh.param("odom_topic", odom_topic, std::string("/odom"));
  pnh.param("reflector_pose_topic", reflector_pose_topic, std::string("reflector_result"));
  pnh.param("laser_frame", laser_frame_, std::string("laser_scanner_link_head"));
  pnh.param("odom_frame", odom_frame_, std::string("odom"));
  pnh.param("base_frame", base_frame_, std::string("base_link"));
  pnh.param("global_frame", global_frame_, std::string("map"));

  scan_sub_ = nh.subscribe(laser_topic, 3, &RosInterface::CallBackLaserScan, this);
  odom_sub_ = nh.subscribe(odom_topic, 3, &RosInterface::CallBackOdometry, this);
  imu_sub_ = nh.subscribe("imu", 3, &RosInterface::CallBackImu, this);
  signal_add_reflector_sub_ = nh.subscribe("signal_add_reflector", 1, &RosInterface::CallBackAddLocalReflector, this);
  signal_save_map_sub_ = nh.subscribe("signal_save_map", 1, &RosInterface::CallBackSaveMap, this);

  localize_result_pub_ = nh.advertise<geometry_msgs::PointStamped>(reflector_pose_topic, 2, true);
  visualizer_hdr_ = std::make_shared<Visualizer>(pnh, true);
  laser_pose_[0] = laser_pose_[1] = laser_pose_[2] = 0.0;
  odom_in_map_.setIdentity();
  tf_broadcaster_.sendTransform(tf::StampedTransform(odom_in_map_, ros::Time::now(), global_frame_, odom_frame_));
}

void RosInterface::RunMain()
{
}

void RosInterface::CallBackMapping(const std_msgs::Bool &msg)
{
  if(msg.data){
    landmark_localizer_hdr_->SetState(State::kMapping);
  }
}

void RosInterface::CallBackMappingEnd(const std_msgs::Bool &msg)
{
  if(msg.data){
    landmark_localizer_hdr_->SetState(State::kLocalizing);
  }
}

void RosInterface::CallBackSaveMap(const std_msgs::Bool &msg)
{
  if(msg.data){
    landmark_localizer_hdr_->SaveMap(kMapPath);
  }
}

void RosInterface::CallBackAddLocalReflector(const std_msgs::Bool &msg)
{
  if(msg.data){
    landmark_localizer_hdr_->AddAllLocalReflectorToMap();
  }
}

void RosInterface::CallBackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  tf_broadcaster_.sendTransform(tf::StampedTransform(odom_in_map_, ros::Time::now(), global_frame_, odom_frame_));
  tf::StampedTransform laser_in_global;
  if(!LookupTF(global_frame_, laser_frame_, ros::Time::now(), laser_in_global)){
    ROS_ERROR("查询激光雷达在地图坐标系下的坐标失败.");
    return;
  }else{
    laser_pose_[0] = laser_in_global.getOrigin().getX();
    laser_pose_[1] = laser_in_global.getOrigin().getY();
    laser_pose_[2] = tf::getYaw(laser_in_global.getRotation());
  }

  landmark_localizer_hdr_->AddLaserScan(msg, laser_pose_);

  Pose2D result;
  Result ret = landmark_localizer_hdr_->GetResult(result);

  // 4. visualize
  // 4.1 visualize the result in laser frame
  visualizer_hdr_->VisualObservedReflector(landmark_localizer_hdr_->GetLocalReflectors(), msg->header.frame_id, 2*kReflectorRadius);
  // 4.2 visualize the match result in laser frame
  visualizer_hdr_->VisualTransedReflector(landmark_localizer_hdr_->GetLocalReflectors(), global_frame_, 5*kReflectorRadius);
  // 4.3 visualize the cluster points in laser frame
  visualizer_hdr_->VisualClusters(landmark_localizer_hdr_->GetClusters(), msg->header.frame_id);
  // 4.4 visualze the filted scan msgs
  visualizer_hdr_->VisualLaserScan(landmark_localizer_hdr_->GetFilteredScan());

  if(Result::kSolveSuccess == ret){
    PublishResult(result);

    tf::Transform laser_in_map;
    laser_in_map.setOrigin(tf::Vector3(result[0], result[1], 0.0));
    laser_in_map.setRotation(tf::createQuaternionFromYaw(result[2]));
    tf::StampedTransform laser_in_odom;
    if(!LookupTF(odom_frame_, laser_frame_, ros::Time::now(), laser_in_odom)){
      ROS_ERROR("查询机器人在里程坐标系下的坐标失败.");
    }else{
      odom_in_map_ = laser_in_map * laser_in_odom.inverse();
      //tf_broadcaster_.sendTransform(tf::StampedTransform(odom_in_map_, ros::Time::now(), global_frame_, odom_frame_));
    }
  }else{
    return;
  }
}

void RosInterface::CallBackOdometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  landmark_localizer_hdr_->AddOdometryData(msg);
}

void RosInterface::CallBackImu(const sensor_msgs::Imu::ConstPtr &msg)
{
  landmark_localizer_hdr_->AddImuData(msg);
}

void RosInterface::PublishResult(const Pose2D &data)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.point.x = data[0];
  msg.point.y = data[1];
  msg.point.z = data[2];
  localize_result_pub_.publish(msg);
}

void RosInterface::SendTransform(const Pose2D &data)
{
  geometry_msgs::TransformStamped stamped_transform;
  stamped_transform.header.frame_id = global_frame_;
  stamped_transform.child_frame_id = odom_frame_;
  stamped_transform.transform = ToGeometryMsgTransform(data);

  tf_broadcaster_.sendTransform(stamped_transform);
}

geometry_msgs::Transform RosInterface::ToGeometryMsgTransform(const Pose2D &pose) const
{
  geometry_msgs::Transform transform;
  transform.translation.x = pose[0];
  transform.translation.y = pose[1];
  transform.translation.z = 0.0;
  transform.rotation = tf::createQuaternionMsgFromYaw(pose[2]);
  return transform;
}

const bool RosInterface::LookupTF(const std::string &target_frame, const std::string &source_frame, const ros::Time &time, tf::StampedTransform &transform) const
{
  bool success = tf_listenner_.waitForTransform(target_frame, source_frame, time, ros::Duration(0.1));
  if(success){
    try
    {
      tf_listenner_.lookupTransform(target_frame, source_frame, time, transform);
    }
    catch(tf::LookupException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    catch(tf::ConnectivityException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    catch(tf::InvalidArgument &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    return true;
  }

  return false;
}



}
