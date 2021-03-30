
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

using namespace multiway;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multiway_navigation");
    ros::NodeHandle nh;

    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    RosInterface node(true, &tf_buffer);

    ros::spin();

    return 0;
}
