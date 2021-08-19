#!/bin/zsh

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)
roslaunch $script_dir/camera_stereo.launch

