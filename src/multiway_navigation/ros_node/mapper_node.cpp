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

#include "ros_node/mapper_node.h"
#include "base/math.h"

namespace multiway {

const std::string kMapPath = std::string(getenv("HOME")) + "/.multiway/map/reflector_map.xml";

MapperNode::MapperNode()
  :reflector_recognizer_hdr_(std::make_shared<ReflectorRecognizer>())
  , landmark_map_hdr_(std::make_shared<LandMarkMap>())
  , least_square_solver_(std::make_shared<LeastSquareSolver>())
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

  scan_sub_ = nh.subscribe(laser_topic, 3, &MapperNode::ScanCallBack, this);
  odom_sub_ = nh.subscribe(odom_topic, 3, &MapperNode::OdomCallBack, this);
  imu_sub_ = nh.subscribe("imu", 3, &MapperNode::ImuCallBack, this);
  signal_add_reflector_sub_ = nh.subscribe("signal_add_reflector", 1, &MapperNode::CallBackAddLocalReflector, this);
  signal_save_map_sub_ = nh.subscribe("signal_save_map", 1, &MapperNode::CallBackSaveMap, this);

  localize_result_pub_ = nh.advertise<geometry_msgs::PointStamped>(reflector_pose_topic, 2, true);

  laser_pose_[0] = laser_pose_[1] = laser_pose_[2] = 0.0;

  // for visulize the result
  visualizer_hdr_ = std::make_shared<Visualizer>(pnh, true);

  tf::StampedTransform t_laser_in_base;
  if(!LookupTF(tf_listenner_, base_frame_, laser_frame_, ros::Time(0), t_laser_in_base)){
    ROS_ERROR("查询雷达在机器人上的坐标转换失败.");
    //laser_in_robot_.setOnes(3);
  }else{
    ROS_DEBUG("查询雷达在机器人上的坐标转换成功.");
    laser_in_robot_ = Pose2Trans(Pose2D(t_laser_in_base.getOrigin().getX(), t_laser_in_base.getOrigin().getY(),
                                        tf::getYaw(t_laser_in_base.getRotation())));
  }


  // 1. load the reflector map
  if(!LoadMap(kMapPath))
  {
    ROS_ERROR("Load reflector map failed!");
  }

  odom_in_map_.setIdentity();
  tf_broadcaster_.sendTransform(tf::StampedTransform(odom_in_map_, ros::Time::now(), global_frame_, odom_frame_));
}

MapperNode::~MapperNode()
{

}

void MapperNode::SaveMap(const std::string &file_name)
{
  // write to xml file
  std::ofstream ofs(file_name);
  if(!ofs.good())
  {
    ROS_INFO("Failed to save landmark map to %s , the reason is cannot open it ", file_name);
    return;
  }
  boost::archive::xml_oarchive oa(ofs);
  oa << BOOST_SERIALIZATION_NVP(landmark_map_hdr_);
  ofs.close();
  ROS_INFO("Save landmark map success.");
}

void MapperNode::AddAllLocalReflectorToMap()
{
  // 这个地方是否需要增加一个选项 --- 是否更新原有地图 ？
  int insert_number = landmark_map_hdr_->AutoIntertIterms(observed_reflectors_);
  ROS_INFO("Insert %d reflectors to reflector map.", insert_number);
}

void MapperNode::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // 0. send tf & laser pose
  tf_broadcaster_.sendTransform(tf::StampedTransform(odom_in_map_, ros::Time::now(), global_frame_, odom_frame_));
  tf::StampedTransform laser_in_global;
  if(!LookupTF(tf_listenner_, global_frame_, laser_frame_, ros::Time::now(), laser_in_global)){
    ROS_ERROR("查询激光雷达在地图坐标系下的坐标失败.");
  }else{
    laser_pose_[0] = laser_in_global.getOrigin().getX();
    laser_pose_[1] = laser_in_global.getOrigin().getY();
    laser_pose_[2] = tf::getYaw(laser_in_global.getRotation());
  }

  // 1. push data to filter
  reflector_recognizer_hdr_->AddLaserScan(msg);

  // 2. 识别反光板在激光雷达坐标系下的坐标
  std::vector<LandMark> local_reflectors = reflector_recognizer_hdr_->GetLocalReflectors();

  if(local_reflectors.size() <= 0)
  {
    ROS_ERROR("Not find reflector in this scan");
    return;
  }

  ROS_DEBUG("Find reflector number: %d", local_reflectors.size());

  // 2.1 将激光雷达坐标系下的反光板，转到地图坐标系
  if(!TransLocalToGlobal(local_reflectors)){
    ROS_ERROR("Failed to trans local point to global point.");
  }

  observed_reflectors_ = local_reflectors;

  // 4. visualize
  // 4.1 visualize the result in laser frame
  visualizer_hdr_->VisualObservedReflector(local_reflectors, msg->header.frame_id, 2*kReflectorRadius);
  // 4.2 visualize the match result in laser frame
  visualizer_hdr_->VisualTransedReflector(local_reflectors, global_frame_, 5*kReflectorRadius);
  // 4.3 visualize the cluster points in laser frame
  visualizer_hdr_->VisualClusters(reflector_recognizer_hdr_->GetClusters(), msg->header.frame_id);
  // 4.4 visualze the filted scan msgs
  visualizer_hdr_->VisualLaserScan(reflector_recognizer_hdr_->GetFilteredScan());

  // 5. 更新定位
  if(landmark_map_hdr_->GetRelectorNum() >= 3)
  {
    // 5.1 遍历地图，搜索匹配的反光板(需要优化算法，加速匹配) <local_id, global_id>
    std::vector<std::pair<LocalLandMark, GlobalLandMark> > matched_reflectors_ = FindMatchedReflectors(local_reflectors);

    // 5.2 如果匹配的反光板数量无法求解，报错
    if(matched_reflectors_.size() < 3) {
      ROS_ERROR("匹配到的反光板数量太少: %d，无法定位.", matched_reflectors_.size());
      return;
    }

    // 5.3 根据匹配上的反光板计算车体位姿
    //Pose2D result = Solve(matched_reflectors_, local_reflectors);
    Pose2D result = Solve(matched_reflectors_);

    if(!std::isnan(result[0]) || !std::isnan(result[1]) || !std::isnan(result[2])){
      // 跳变检查
      Pose2D delta_pose = result - laser_pose_;
      static int counter = 0;
      if(counter >=1 && (std::fabs(delta_pose[0]) >= 0.15 || std::fabs(delta_pose[1]) >= 0.15)){
        ROS_ERROR("Position jumped!");
        //return;
      }
      counter++;

      tf::Transform laser_in_map;
      laser_in_map.setOrigin(tf::Vector3(result[0], result[1], 0.0));
      laser_in_map.setRotation(tf::createQuaternionFromYaw(result[2]));
      tf::StampedTransform laser_in_odom;
      if(!LookupTF(tf_listenner_, odom_frame_, laser_frame_, ros::Time::now(), laser_in_odom)){
        ROS_ERROR("查询机器人在里程坐标系下的坐标失败.");
      }else{
        odom_in_map_ = laser_in_map * laser_in_odom.inverse();
        //tf_broadcaster_.sendTransform(tf::StampedTransform(odom_in_map_, ros::Time::now(), global_frame_, odom_frame_));
      }
      PublishResult(result);
      //laser_pose_ = result;
      ROS_INFO("求解成功，更新姿态.");
    }else{
      ROS_ERROR("求解失败");
    }
  }else
  {
    ROS_ERROR("Not update localization");
  }

}

void MapperNode::OdomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  //laser_corrector_hdr_->AddOdometryData(*msg);

  /*
  if(std::fabs(msg->twist.twist.linear.x) <= 1e-3 && std::fabs(msg->twist.twist.angular.z) <= 1e-3){
    mean_filter_hdr_->SetCachedScanNum(30);
  }else{
    mean_filter_hdr_->SetCachedScanNum(1);
  }
  */
}

void MapperNode::ImuCallBack(const sensor_msgs::Imu::ConstPtr &msg)
{
  //laser_corrector_hdr_->AddImuData(*msg);
}

void MapperNode::CallBackAddLocalReflector(const std_msgs::Bool &msg)
{
  if(true == msg.data)
  {
    ROS_INFO("Add local reflector to map.");
    AddAllLocalReflectorToMap();
  }
}

void MapperNode::CallBackSaveMap(const std_msgs::Bool &msg)
{
  if(true == msg.data)
  {
    ROS_INFO("Save map to: %s.", kMapPath.c_str());
    SaveMap(kMapPath);
  }
}

const bool MapperNode::TransLocalToGlobal(std::vector<LandMark> &local_reflectors)
{
  //
  // 1. get the robot in map trans
  tf::StampedTransform t_laser_in_map;
  if(!LookupTF(tf_listenner_, global_frame_, laser_frame_, ros::Time(0), t_laser_in_map)) {
    ROS_ERROR("查询激光雷达在地图坐标系下的坐标失败");
    return false;
  }
  Eigen::Matrix3d laser_in_map = Pose2Trans(Pose2D(t_laser_in_map.getOrigin().getX(), t_laser_in_map.getOrigin().getY(),
                                                   tf::getYaw(t_laser_in_map.getRotation())));

  // 2. trans the local to global
  for(auto iter = local_reflectors.begin(); iter != local_reflectors.end(); iter++) {
    iter->UpdateGlobalPos(laser_in_map);
  }
  return true;
}

const std::vector<std::pair<LocalLandMark, GlobalLandMark> > MapperNode::FindMatchedReflectors(std::vector<LandMark>& local_reflectors) const
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

// <local_id, global_id>
const Pose2D MapperNode::Solve(const std::vector<std::pair<LocalLandMark, GlobalLandMark> > &matched_reflctor) const
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
  least_square_solver_->SetInput(pair_points, laser_pose_);
  Pose2D ret = least_square_solver_->ComputeResult();

  return ret;
}

bool MapperNode::LoadMap(const std::string &file_name)
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

void MapperNode::PublishResult(const Pose2D &data)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.point.x = data[0];
  msg.point.y = data[1];
  msg.point.z = data[2];
  localize_result_pub_.publish(msg);
}

const bool MapperNode::LookupTF(const tf::TransformListener &tf_listener,
                          const std::string &target_frame, const std::string &source_frame,
                          const ros::Time &time, tf::StampedTransform &transform) const
{
  bool success = tf_listener.waitForTransform(target_frame, source_frame, time, ros::Duration(0.1));
  if(success){
    try
    {
      tf_listener.lookupTransform(target_frame, source_frame, time, transform);
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
