# ROS2 RIEGL Controller
[ros-riegl-vz](https://github.com/riegllms/ros-riegl-vz)

## Configuration
The configuration file for the scanner is `install/riegl_vz/share/riegl_vz/config/params.yaml` older configurations are saved in `configs`.


## Commands
For documentation, some commands that have previously been used:
```bash
ros2 service call /scan std_srvs/srv/Trigger # start scan
ros2 service call /get_scan_poses riegl_vz_interfaces/srv/GetScanPose # get position
ros2 topic echo /gnss
ros2 topic echo /pose
```

