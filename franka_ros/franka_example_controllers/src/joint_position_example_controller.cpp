// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  // for (size_t i = 0; i < q_start.size(); i++) {
  //   if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionExampleController: Robot is not in the expected starting position for "
  //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //     return false;
  //   }
  // }

  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
  _joints_position_goal = initial_pose_;
  _joints_position_command = initial_pose_;

  for (int i = 0; i < 7; ++i) {
    _joints_velocity_goal[i] = 0.02;
  }
  _joints_position_goal[0] += M_PI_4;


  double d = 100.0;
  _joints_acceleration_limits = {15.0/d, 7.5/d, 10.0/d, 12.5/d, 15.0/d, 20.0/d, 20.0/d};
}

void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  //Total time passed since controller started converted to the double.
  double seconds_passed = elapsed_time_.toSec();

  //Set distance to goal point to 0 in order to check with each new loop.
  double distance_to_goal_point = 0;


  // For each joint check if they are at goal position or move them to the goal position
  for (size_t i=0; i<7; ++i)
  {
    // Distance between goal position and joint's current position
    distance_to_goal_point = _joints_position_goal[i] - position_joint_handles_[i].getPosition();

    //Since joints are turning both -/+ sides, we need it as a multiplier.
    int direction = std::signbit(distance_to_goal_point)==1?-1:1;

    //If distance_to_goal_point is more than this value keep joints working. Less than 0.001 causes problems becareful with oscillations.
    double goal_distance_check_value = 0.001;

    // Seconds_passed prevents oscillations at the first call of the controller. DO NOT DELETE!
    if (std::fabs(distance_to_goal_point)>goal_distance_check_value || seconds_passed <= 0.001)
    {
      double time_needed_to_stop_with_current_vel_and_acc = position_joint_handles_[i].getVelocity() / _joints_acceleration_limits[i];
      double distance_needed_to_stop_with_current_vel = _joints_acceleration_limits[i]*time_needed_to_stop_with_current_vel_and_acc*time_needed_to_stop_with_current_vel_and_acc/2;

      //Deceleration
      if ((( direction == -1 & position_joint_handles_[i].getPosition() < _joints_position_goal[i] + distance_needed_to_stop_with_current_vel) ||
        ( direction == 1 & position_joint_handles_[i].getPosition() > _joints_position_goal[i] - distance_needed_to_stop_with_current_vel))
          & std::fabs(position_joint_handles_[i].getVelocity()) > 0.017
        )
      {
        _current_joint_velocity_commands[i] -= (direction) * _joints_acceleration_limits[i]*period.toSec();
      }
      //Acceleration
      else if (std::fabs(_joints_velocity_goal[i] - std::fabs(position_joint_handles_[i].getVelocity())) > 0.017 )
      {
        _current_joint_velocity_commands[i] += (direction) * _joints_acceleration_limits[i]*period.toSec();
      }
      //Constant Velocity
      else{
        _current_joint_velocity_commands[i] = (direction) * _joints_velocity_goal[i];
      }

      _joints_position_command[i] += (direction)* _joints_velocity_goal[i]*period.toSec();
      position_joint_handles_[i].setCommand(_joints_position_command[i]);
    }
  }

}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
