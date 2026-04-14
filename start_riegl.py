#!/usr/bin/env python3

from glob import glob
from sys import argv
import subprocess

project_name = argv[1]
files = glob(f"params/{project_name}_*.yaml")
files = sorted(files)[-1]
project_number = int(files.split('/')[-1].split('.')[0].split('_')[-1])
project_number += 1
new_file_path = f"params/{project_name}_{project_number}.yaml"

with open(new_file_path, 'w') as f:
    f.write(f'''riegl_vz_node:
  ros__parameters:
    hostname: "10.0.1.1"
    ssh_user: "user"
    ssh_password: "user"
    working_dir: "/tmp/ros_riegl_vz"
    project_name: "{project_name}_{project_number}"
    scanpos_name: ""
    storage_media: 0
    meas_program: 3
    scan_pattern_name: ""
    scan_pattern: [30.0, 130.0, 0.04, 0.0, 360.0, 0.04]
    scan_publish: False
    scan_publish_filter: ""
    scan_publish_lod: 8
    scan_register: True
    scan_registration_mode: 1
    pose_publish: True
    pose_publish_fast: True
    voxel_publish: True
    reflector_search: False
    reflector_search_models: "RIEGL flat reflector 50 mm"
    reflector_search_limits: [1.0, 100.0]
    control_points_csv_file_remote: ""
    control_points_csv_file_local: ""
    control_points_coord_system: "EPSG::4937"
    image_capture: 0
    image_capture_mode: 1
    image_capture_overlap: 25
    set_pose_topic: ""
    robot_relative_pose: False
    robot_scanner_mounting: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot_project_frame_id: ""
    robot_scanner_project_transform: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    imu_data_publish: False
    imu_index: 0''')

try:
    subprocess.run(['./apply_params.sh', new_file_path])
    subprocess.run(['ros2', 'launch', 'riegl_vz', 'std_launch.py'])
except KeyboardInterrupt:
    pass
