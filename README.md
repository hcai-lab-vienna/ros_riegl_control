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

## Notes


### Approximate position based on 3 key data points
- tf frame `ros2 run tf2_ros tf2_echo base_link odom` python (odom from robot)
- pose topic from riegl scanner
- gnss signal from riegl scanner


- [slam ros2](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [tf2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [kiss icp](https://github.com/PRBonn/kiss-icp) [arxiv.org/abs/2209.15397](https://arxiv.org/abs/2209.15397)
- [odometry ros2 scout data](https://github.com/agilexrobotics/scout_ros) - Alternative odometry calculation
