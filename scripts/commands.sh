#!/bin/bash

scout_move() {
	ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}"
	echo "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}"
}

move_one_meter() {
	for i in {1..22}; do
		scout_move "0.125" "0.0"
	done
}

turn_left() {
	for i in {1..15}; do
		scout_move "0.0" "0.25"
	done
}

turn_right() {
	for i in {1..15}; do
		scout_move "0.0" "-0.25"
	done
}

turn_left
turn_right
