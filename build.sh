#!/bin/bash
git submodule update --init --recursive
./fixes/fix_riegl_typo.sh
colcon build
