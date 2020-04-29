// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

class JointPositionExampleControllerSim :  public controller_interface::Controller<hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) ;
  void starting(const ros::Time&) ;
  void update(const ros::Time&, const ros::Duration& period) ;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};

  ros::Subscriber command_sub_;
  std::vector<double> command_;
  void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
};

}  // namespace franka_example_controllers
