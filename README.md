# ROS2 RIEGL Controller

## Install
Follow [ros-riegl-vz](https://github.com/riegllms/ros-riegl-vz)
Requirements for ROS2 jazzy:
```bash
sudo apt install ros-jazzy-tf2-tools ros-jazzy-tf-transformations python3-pip python3-paramiko python3-scp python3-protobuf python3-empy python3-lark python3-numpy python3-transforms3d
```

And clone repo with all submodules:
```bash
git clone --recurse-submodules https://github.com/hcai-lab-vienna/ros_bunker_control.git
```
or if you cloned the repo before reading this, download all submodules like this:
```bash
git submodule update --init --remote --recursive
```

Then do:
```bash
pip3 install --user --break-system-packages src/ros-riegl-vz/librdb/riegl.rdb-2.5.3-py3-none-linux_x86_64.whl
```
and run the following script in project root to fix a issue in the riegl repo:
```bash
./fix_riegl_typo.sh
```


## Configuration
The configuration file for the scanner is `install/riegl_vz/share/riegl_vz/config/params.yaml` older configurations are saved in `configs`.

Apply new configs after compilation from project root with:
```bash
./apply_params.sh <path to a params.yaml>
```


## Commands
For documentation, some commands that have previously been used:
```bash
ros2 launch riegl_vz std_launch.py # start scanner server
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



# ROS2 Bunker Control


## Setup

### Download

Requirments are [ROS2 jazzy](https://docs.ros.org/en/jazzy/Installation.html) (newer should also work) and:
```bash
sudo apt install build-essential git cmake libasio-dev can-utils
```
and
```bash
sudo apt install ros-jazzy-nav2-msgs
```

First clone repo with all submodules:
```bash
git clone --recurse-submodules https://github.com/hcai-lab-vienna/ros_bunker_control.git
```
or if you cloned the repo befor reading this, download all submodules like this:
```bash
git submodule update --init --remote --recursive
```
or follow the instruction from [agilexrobotics/bunker_ros2](https://github.com/agilexrobotics/bunker_ros2) for `src`.

---

Also init CAN2USB adapter once with, if not done before:
```bash
bash src/ugv_sdk/scripts/setup_can2usb.bash
```

### Build

In the root directory of the repository after downloading all submodules with an active ros environment do:
```bash
colcon build
```

For troubleshooting see:
- [agilexrobotics/bunker_ros2](https://github.com/agilexrobotics/bunker_ros2)
- [agilexrobotics/ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)


## Run

Connect Bunker via CAN2USB adapter and then do in the root of the repo after building:
```bash
source install/setup.bash
bash src/ugv_sdk/scripts/bringup_can2usb_500k.bash
ros2 launch bunker_base bunker_base.launch.py
```
or just run the `start_bunker_base.sh` script.
