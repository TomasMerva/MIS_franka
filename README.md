# MIS_franka

### Instalation
`sudo apt install ros-noetic-libfranka` -> you do not need it for slave PC


### ROS Master-Slave connection
1. Set IP address on your **slave PC** 192.16.0.2
2. Set these two commands in `~/.bashrc` on your **slave PC**:

`export ROS_MASTER_URI=http://192.16.0.1:11311`

`export ROS_IP=192.16.0.2`

### Connect to the master PC
`ssh -X km@192.16.0.1`


### Start guide
Launch Franka cartesian_velocity_controller and force_controller on **master PC**:

`roslaunch franka_example_controllers cartesian_velocity_example_controller.launch`

Launch joystick node (joystick driver + joystick_EEF_controller) on **slave PC**:

`roslaunch franka_joystick_control joystick_control.launch`

### Important topics
`/joystick_feedback` -> Raw data from joystick. For more info check `/franka_joystick_control/src/joystick_pub.cpp`

`/joystick_cartesian_goal` -> Commands for Franka representing cartesian velocities



### Acknowledgement
[franka_ros](https://github.com/frankaemika/franka_ros)


