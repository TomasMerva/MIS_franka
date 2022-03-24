// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }


  _cartesian_goal_sub = node_handle.subscribe("/joystick_cartesian_goal", 1, &CartesianVelocityExampleController::CartesianGoalCallback, this);

  return true;
}

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  for (size_t i = 0; i < 7; ++i) {
      _cartesian_velocity_goal[i] = 0.0;
      _cartesian_velocity_prev[i] = 0.0;
  }
}

void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  ros::spinOnce();
  elapsed_time_ += period;

  double filter_value=0.01;

  for (size_t i=0; i<6; i++)
  {
    _cartesian_velocity_prev[i] = filter_value*_cartesian_velocity_goal[i] + (1.0-filter_value)*_cartesian_velocity_prev[i];
  }
  velocity_cartesian_handle_->setCommand(_cartesian_velocity_prev);
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityExampleController::CartesianGoalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  _cartesian_velocity_goal[0] = msg->linear.x;
  _cartesian_velocity_goal[1] = msg->linear.y;
  _cartesian_velocity_goal[2] = msg->linear.z;
  // std::cout << "msg: " << msg->linear.z << std::endl;
  // std::cout << "command" << _cartesian_velocity_goal[2] << std::endl;
}

}  // namespace franka_example_controllers




PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)
