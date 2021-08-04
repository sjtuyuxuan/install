#!/bin/zsh
roslaunch ouster_ros ouster.launch sensor_hostname:=192.168.1.2 udp_dest:=192.168.1.100 metadata:=$HOME/Documents/Ouster/temp.json lidar_mode:=2048x10 timestamp_mode:=TIME_FROM_SYNC_PULSE_IN sync_pulse_in_polarity:=ACTIVE_HIGH viz:=false
