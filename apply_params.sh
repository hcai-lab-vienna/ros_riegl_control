#!/bin/bash

# if [ $# -eq 0 ]; then
# 	echo "please provided param file as argument, they should be located at ./params"
# fi
# SOURCE_FILE="$1"

# RIEGL
SOURCE_FILE="params/riegl/params.yaml"
TARGET_PATH="install/riegl_vz/share/riegl_vz/config/"
TARGET_FILE="install/riegl_vz/share/riegl_vz/config/params.yaml"
if [ -f "$SOURCE_FILE" ]; then
	mkdir -p "$TARGET_PATH"
	cp "$SOURCE_FILE" "$TARGET_FILE"
else
	echo "$SOURCE_FILE does not exist."
fi

# HELIOS
SOURCE_FILE="params/helios/config.yaml"
TARGET_FILE="src/rslidar_sdk/config/config.yaml"
if [ -f "$SOURCE_FILE" ]; then
	cp "$SOURCE_FILE" "$TARGET_FILE"
else
	echo "$SOURCE_FILE does not exist."
fi
