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


#include "base/landmark_map.h"

namespace multiway {

bool LandMarkMap::InsertItem(const std::pair<uint, LandMark> &item)
{
  // if there is a repeat item, delete it and use new to replace it
  if(reflector_list_.find(item.first) != reflector_list_.end()) {
    reflector_list_.erase(reflector_list_.find(item.first));
  }
  std::pair<std::map<uint, LandMark>::iterator, bool> ret = reflector_list_.insert(item);
  if(ret.second) return true;
  else return false;
}

const uint LandMarkMap::AutoIntertIterms(const std::vector<LandMark> &items)
{
  uint insert_number = 0;

  for(auto iter = items.begin(); iter != items.end(); iter++)
  {
    if(FindVeryNearItem(*iter) < 0)
    {
      if(InsertItem(std::pair<uint, LandMark>(reflector_list_.size()+1, *iter)))
        insert_number++;
    }
  }
  return insert_number;
}

int LandMarkMap::FindVeryNearItem(const LandMark &land_mark, const double &threshold)
{
  // check if there is a very near item in landmark map list 
  double min_dis = 30.0;
  int near_id = -1;
  for(auto iter = reflector_list_.begin(); iter != reflector_list_.end(); iter++)
  {
    double distance = land_mark.DistanceTo(iter->second);
    if(distance < min_dis)
    {
      min_dis = distance;
      near_id = iter->first;
    }
  }

  if(min_dis < threshold)
    return near_id;
  else
    return -1;
}

const bool LandMarkMap::GetLandMarkByID(const uint &id, LandMark &land_mark) const
{
  auto iter = reflector_list_.find(id);
  if(iter != reflector_list_.end()){
    land_mark = iter->second;
    return true;
  }
  return false;
}

const LandMark LandMarkMap::GetLandMarkByID(const uint &id)
{
  LandMark ret;
  auto iter = reflector_list_.find(id);
  if(iter != reflector_list_.end()){
    ret = iter->second;
  }
  return ret;
}

const double LandMarkMap::DistanceBetweenItem(const LandMark &a, const LandMark &b)
{
  return std::sqrt(pow(a.GetLocalX() - b.GetLocalX(), 2) + pow(a.GetLocalY() - b.GetLocalY(), 2));
}

}
