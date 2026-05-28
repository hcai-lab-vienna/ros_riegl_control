#!/bin/bash

rm -rf build install log

git submodule update --init --recursive

colcon build --packages-ignore hcai_nav_bringup basic_pure_pursuit_controller ros_imu

colcon build --symlink-install --packages-select hcai_nav_bringup basic_pure_pursuit_controller ros_imu
