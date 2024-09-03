/**
 * @file sport_mode_control.hpp
 * @brief
 * @date 28-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef SPORT_MODE_CONTROL_HPP
#define SPORT_MODE_CONTROL_HPP

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_ros2_client/common/ros2_sport_client.h"

enum sport_modes {
  kIdle = 0,
  kBalanceStand,
  kPose,
  kLocomotion,
  kReserve1,
  kLieDown,
  kJointLock,
  kDamping,
  kRecoveryStand,
  kReserve2,
  kSit,
  kFrontFlip,
  kFrontJump,
  kFrontPounce
};

class SportModeControl : public rclcpp::Node {
 public:
  SportModeControl();
  void SetupROSInterfaces();

  // Callbacks
  void SportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
  void TimerCallback();

 private:
  SportClient sport_client_handle_;
  unitree_api::msg::Request req_;

  int mode_;
  int control_rate_;

  // Publisher
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_mode_ctrl_pub_;

  // Subscriber
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* SPORT_MODE_CONTROL_HPP */