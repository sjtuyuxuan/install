#!/bin/zsh

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)
while getopts "r:s" optname
do
    case "$optname" in
    "r")
        framerate=$OPTARG
        ;;
    "s")
        sycn=true
        ;;
    esac
done
if [ $framerate ];
    then
    roslaunch $script_dir/camera.launch frame_rate:=$framerate
elif [ $sycn ];
    then
    roslaunch $script_dir/camera.launch control_frame_rate:=false enable_trigger:=On
else
    roslaunch $script_dir/camera.launch
fi

