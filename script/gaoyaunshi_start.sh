#!/bin/zsh
rosbag record -o "/home/mpl/DATA/data" /camera/image_mono /camera/image_mono/compressed /os_cloud_node/points /imu/data -b 12288

