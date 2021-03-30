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
 * @brief
 */

#pragma once

#include <map>
#include <boost/serialization/map.hpp>
#include "base/land_mark.h"

namespace multiway {

class LandMarkMap;
using LandMarkMapPtr = LandMarkMap*;
using LandMarkMapCPtr = const LandMarkMap*;
using LandMarkMapHdr = std::shared_ptr<LandMarkMap>;

class LandMarkMap final
{
public:
  LandMarkMap() = default;
  ~LandMarkMap() = default;

  /**
   * @brief InserItem insert item to the reflector map, if the ID is repeat, delete it, and use new
   * @param item now is reflector item
   * @return true if insert succes, false other wise
   */
  bool InsertItem(const std::pair<uint, LandMark>& item);

  /**
   * @brief AutoInterIterms automatic insert all local items
   * @param items items
   * @return successly insert number
   */
  uint AutoIntertIterms(const std::vector<LandMark>& items);

  /**
   * @brief IsHaveVeryNearItem check if there is a very near item in landmark map list
   * @param land_mark Input land mark, that need to compare
   * @param threshold Less than this threshold we think this is a repeat reflector
   * @return -1 if no very near item,  item ID other wise
   */
  int FindVeryNearItem(const LandMark& land_mark, const double& threshold = 0.4);

  /**
   * @brief ClearItem Clear map
   */
  inline void ClearItem() { reflector_list_.clear(); }

  /**
   * @brief GetRelectorNum get the reflector
   * @return
   */
  inline int GetRelectorNum() const { return reflector_list_.size(); }

  /**
   * @brief GetLandMarkList return the landmark list
   * @return landmark list
   */
  inline std::map<uint, LandMark> GetLandMarkList() const { return reflector_list_; }

  bool _rt GetLandMarkByID(const uint& id, LandMark& land_mark) const;

  LandMark _rt GetLandMarkByID(const uint& id);

private:
  double _rt DistanceBetweenItem(const LandMark& a, const LandMark& b);

private:
  std::map<uint, LandMark> reflector_list_;

  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(reflector_list_);
  }
};

} // end of namespace multiway
