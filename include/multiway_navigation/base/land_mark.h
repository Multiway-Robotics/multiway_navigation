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
 * @brief LandMark class to present the reflector position in specific frame
 */

#pragma once

#include <fstream>
#include <boost/serialization/access.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/nvp.hpp>

#include <boost/serialization/scoped_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/shared_ptr_helper.hpp>
#include "base/macros.h"
#include "base/data_structs.h"

namespace multiway {

class LandMark;
using LandMarkPtr = LandMark*;
using LandMarkHdr = std::shared_ptr<LandMark>;
using LocalLandMark = LandMark;
using GlobalLandMark = LandMark;

constexpr float kReflectorRadius = 0.045;
constexpr float kReflectorRadiusEsp = 0.0;

class LandMark final
{
public:
  LandMark(): local_id_(0) ,local_x_(0.0), local_y_(0.0), radius_(0.0) { }
  LandMark(uint32_t _in id, Cluster _in cluster, double _in radius = 0.09);
  ~LandMark(){}

  void Init();

  inline uint32_t GetID() const { return local_id_; }
  inline Point3D LocalPoint() const { return Point3D(local_x_, local_y_, 1.0); }
  inline Point3D GlobalPoint() const { return Point3D(global_x_, global_y_, 1.0); }

  inline double GetLocalX() const { return local_x_; }
  inline double GetLocalY() const { return local_y_; }
  inline double GetGlobalX() const { return global_x_; }
  inline double GetGlobalY() const { return global_y_; }

  inline double GetRadius() const { return radius_; }

  inline Cluster GetCluster() const { return beams_;}

  inline double GetCenterAngle() const { return cluster_center_; }

  inline double GetDistanceInLocalFrame() const { return std::sqrt(pow(local_x_, 2) + pow(local_y_, 2)); }

  inline double DistanceTo(const LandMark& other) const { return std::sqrt(pow(global_x_ - other.GetGlobalX(), 2) + pow(global_y_ - other.GetGlobalY(), 2)); }

  inline double SquareDistanceTo(const LandMark&  other) const { return pow(global_x_ - other.GetGlobalX(), 2) + pow(global_y_ - other.GetGlobalY(), 2); }

  inline void SetLandMark(const Point3D& p, const double& radius = 0.09) {
    local_x_ = p[0];
    local_y_ = p[1];
    radius_ = radius;
  }

  void UpdateGlobalPos(const Eigen::Matrix3d& local_in_global);

  inline void SetCluster(const Cluster& cluster) { beams_ = cluster; }

  inline void SetClusterCenter(const double& angle) { cluster_center_ = angle; }

  bool operator<(LandMark const& other) const { return this->GetDistanceInLocalFrame() < other.GetDistanceInLocalFrame(); }

private:
  uint32_t local_id_;
  double local_x_;
  double local_y_;
  double radius_;
  double global_x_;
  double global_y_;
  Cluster beams_;
  double cluster_center_;

  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(global_x_);
    ar & BOOST_SERIALIZATION_NVP(global_y_);
    ar & BOOST_SERIALIZATION_NVP(radius_);
  }
};

} // end of namespace mutiway
