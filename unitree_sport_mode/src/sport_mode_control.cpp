/**
 * @file sport_mode_control.cpp
 * @brief
 * @date 28-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include "unitree_sport_mode/sport_mode_control.hpp"

SportModeControl::SportModeControl() : Node("SportModeControl") {
  SetupROSInterfaces();
}

void SportModeControl::SetupROSInterfaces() {
  control_rate_ = this->declare_parameter("control_rate", 3000);
  sport_mode_ctrl_pub_ = this->create_publisher<unitree_api::msg::Request>(
      "api/sport/request", 10);
  sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10,
      std::bind(&SportModeControl::SportStateCallback, this,
                std::placeholders::_1));
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_rate_),
      std::bind(&SportModeControl::TimerCallback, this));
}

void SportModeControl::SportStateCallback(
    const unitree_go::msg::SportModeState::SharedPtr msg) {
  mode_ = msg->mode;
  RCLCPP_INFO(this->get_logger(), "Motion mode: %d", mode_);
}

void SportModeControl::TimerCallback() {
  switch (mode_) {
    case sport_modes::kDamping:
      sport_client_handle_.StandUp(req_);
      RCLCPP_INFO(this->get_logger(), "StandUp mode initiated");
      break;
    case sport_modes::kJointLock:
      sport_client_handle_.BalanceStand(req_);
      RCLCPP_INFO(this->get_logger(), "BalanceStand mode initiated");
      break;
    case sport_modes::kBalanceStand:
      sport_client_handle_.StandUp(req_);
      RCLCPP_INFO(this->get_logger(), "StandUp mode initiated");
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Invalid mode initiated");
      break;
  }

  sport_mode_ctrl_pub_->publish(req_);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto smc_node = std::make_shared<SportModeControl>();
  RCLCPP_INFO(smc_node->get_logger(), "Control loop rate: %ldms",
              smc_node->get_parameter("control_rate").as_int());

  RCLCPP_INFO(smc_node->get_logger(),
              "WARNING: Make sure the robot is lying on the ground.");
  RCLCPP_INFO(smc_node->get_logger(), "Press Enter to continue...");
  std::cin.ignore();

  rclcpp::spin(smc_node);
  rclcpp::shutdown();
  return 0;
}