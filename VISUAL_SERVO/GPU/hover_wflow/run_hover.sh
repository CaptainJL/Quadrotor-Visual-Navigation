#!/bin/bash

# maximise performance of tx2
sudo su << HERE
echo 1 > /sys/devices/system/cpu/cpu1/online
echo 1 > /sys/devices/system/cpu/cpu2/online
HERE

sudo ./jetson_clocks.sh

gnome-terminal -e "roscore"
sleep 3
gnome-terminal -e "rosrun GPU_2_Quad_ROS Tegra_2_Pixhawk_serial"
sleep 3
gnome-terminal -e "rosrun hover_wflow flowcontrol"
