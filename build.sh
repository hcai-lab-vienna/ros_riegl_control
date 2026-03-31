#!/bin/bash

rm -rf build install log

git submodule update --init --recursive

./fixes/fix_riegl_typo.sh
colcon build --packages-ignore hcai_nav_bringup basic_pure_pursuit_controller

colcon build --symlink-install --packages-select hcai_nav_bringup basic_pure_pursuit_controller
