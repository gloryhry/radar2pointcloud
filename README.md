# radar2pointcloud

[![Actions Status](https://github.com/gloryhry/radar2pointcloud/workflows/CI/badge.svg)](https://github.com/gloryhry/radar2pointcloud)
[![GitHub Workflow Status](https://img.shields.io/github/workflow/status/gloryhry/radar2pointcloud/CI)](https://github.com/gloryhry/radar2pointcloud/actions)
[![license](https://img.shields.io/badge/license-GPL--3.0-green.svg)](https://github.com/gloryhry/radar2pointcloud/blob/master/LICENSE)

## Instructions for use

Use to transfer radar msgs into pointcloud in ros.

## Dependent

[yaml-cpp](https://github.com/jbeder/yaml-cpp.git)

## Require messages

- [ecal_to_ros](https://github.com/gloryhry/Ecal-to-ROS)
- [gps_msgs](https://github.com/gloryhry/gps_msgs)

## How to use

1. install dependent ([yaml-cpp](https://github.com/jbeder/yaml-cpp.git))
2. run code:
```mkdir catkin_ws
cd catkin_ws && mkdir src
cd src && mkdir Msgs
cd Msgs
git clone https://github.com/gloryhry/gps_msgs.git
git clone https://github.com/gloryhry/Ecal-to-ROS.git
cd ..
git clone https://github.com/gloryhry/radar2pointcloud.git
cd ..
catkin_make
source devel/setup.bash 
roslaunch radar2pointcloud radar2pointcloud.launch
```
3. play rosbag.
