#!/bin/bash
if [ $# -eq 0 ]; then
	echo "please provided param file as argument, they should be located at ./params"
fi
SOURCE_FILE="$1"
TARGET_PATH="install/riegl_vz/share/riegl_vz/config/"
TARGET_FILE="install/riegl_vz/share/riegl_vz/config/params.yaml"
if [ -f "$SOURCE_FILE" ]; then
	mkdir -p "$TARGET_PATH"
	cp "$SOURCE_FILE" "$TARGET_FILE"
else
	echo "$SOURCE_FILE does not exist."
fi
