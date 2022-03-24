cmake_minimum_required(VERSION 3.0.2)
project(franka_joystick_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf2
  trajectory_msgs
  visualization_msgs
  franka_core_msgs
)

### Generate messages in the 'msg' folder
add_message_files(
  FILES
  joystick_msg.msg
)

### Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include/franka_joystick_control
  LIBRARIES joystick_lib franka_joystick_control_lib
  CATKIN_DEPENDS
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    rospy
    std_msgs
    tf2
    trajectory_msgs
    visualization_msgs
    franka_core_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(joystick_lib src/joystick_lib.cpp)
target_link_libraries(joystick_lib ${catkin_LIBRARIES})

add_executable(joystick_publisher src/joystick_pub.cpp )
target_link_libraries(joystick_publisher
                    joystick_lib
                    ${catkin_LIBRARIES})



add_library(franka_joystick_control_lib src/franka_joystick_control_lib.cpp)
target_link_libraries(franka_joystick_control_lib ${catkin_LIBRARIES})

add_executable(franka_joystick_control src/franka_joystick_control.cpp )
target_link_libraries(franka_joystick_control
                      franka_joystick_control_lib
                      ${catkin_LIBRARIES})



