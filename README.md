# HCC-Lab3-Tracking
This code is based on [tello_ros](https://github.com/clydemcqueen/tello_ros). This code use raw_image node at tello_ros and process to track a red ball
The following are the insruction.

To run this code, we need 2 terminals
## Terminal 1
```console=
$ source /opt/ros/foxy/setup.bash
$ cd ~/tello_ros_ws/
$ source install/setup.bash
$ ros2 run tello_driver tello_driver_main  # activate tello_ros raw_image node
```

## Terminal 2
```console=
$ source /opt/ros/foxy/setup.bash
$ cd ~/HCC-Lab3-Tracking/
$ colcon build --symlink-install  # Build the workspace
$ source install/setup.bash
$ ros2 launch tello_tracking tracking_launch.py  # Launch the tracking program
```

RViz will appeared after launching tracking_launch.py, follow the instruction to visualized the tracking result:
1. Fixed frame: choose `camera_frame`
2. Add `ball_trajectory` and `tracking_result_image` by topic
3. You can see the tracking result at RViz


After close Terminal 2 manually by `Ctrl + C`, a file named `ball_trajectory.csv` will be in `~/`
