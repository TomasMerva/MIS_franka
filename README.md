# MIS_franka

### Instalation
1. `sudo apt install ros-noetic-libfranka`

2. `sudo apt install ros-noetic-moveit`

3. `sudo apt-get install ros-noetic-trac-ik-kinematics-plugin`

4. git clone this repo

5. `rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka`

6. build without files in CMakeLists (franka_joystick_control) because of the custom msg

7. build with files in CMakeLists (franka_joystick_control)


### ROS Master-Slave connection
1. Set IP address on your **slave PC** `192.16.0.2`
2. Set these two commands in `~/.bashrc` on your **slave PC**:

`export ROS_MASTER_URI=http://192.16.0.1:11311`

`export ROS_IP=192.16.0.2`

### Connect to the master PC
`ssh -X km@192.16.0.1`

After logging into the master PC you need to release the robot's brakes and allow FCI. By writting `firefox` into terminal,
you will automatically enter `Franka DESK` where you can do these steps.


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


