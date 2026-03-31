#!/bin/bash

rm -rf build install log

git submodule update --init --recursive

colcon build --packages-select ugv_sdk
colcon build --packages-select bunker_ros2

colcon build --packages-select ros-riegl-vz
./fixes/fix_riegl_typo.sh

colcon build --packages-select hcai_nav_bringup --symlink-install
colcon build --packages-select basic_pure_pursuit_controller --symlink-install
