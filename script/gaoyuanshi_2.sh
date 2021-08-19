#!/bin/zsh
rosbag record -o "/home/mpl/DATA/data" /stereo/left/image_mono /stereo/right/image_mono /os_cloud_node/points /imu/data /prophesee/camera_right/cd_events_buffer_reconstructed /prophesee/camera_left/cd_events_buffer_reconstructed /depth/image_raw /rgb/image_raw /points2 -b 12288

