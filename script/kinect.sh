#!/bin/zsh

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)
roslaunch $script_dir/k4a_driver.launch
