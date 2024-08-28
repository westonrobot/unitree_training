/**
 * @file sport_mode_control.hpp
 * @brief
 * @date 28-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef SPORT_MODE_CONTROL_HPP
#define SPORT_MODE_CONTROL_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "common/ros2_sport_client.h"

enum sport_modes {
  idle = 0,
  balanceStand,
  pose,
  locomotion,
  reserve,
  lieDown,
  jointLock,
  damping,
  recoveryStand,
  reserve,
  sit,
  frontFlip,
  frontJump,
  frontPounc
};

class SportModeControl : public rclcpp::Node {
 public:
  SportModeControl();
  void SetupROSInterfaces();

  // Callback
  void SportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);

 private:
  SportClient sport_client_handle_;
  unitree_api::msg::Request req_;
  
  int mode_;

  // Publisher
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_mode_ctrl_pub_;

  // Subscriber
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_sub_;
};

#endif /* SPORT_MODE_CONTROL_HPP */