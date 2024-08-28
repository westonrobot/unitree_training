/**
 * @file sport_mode_control.cpp
 * @brief
 * @date 28-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include "sport_mode_samples/sport_mode_control.hpp"

SportModeControl::SportModeControl() : Node("SportModeControl") {
  SetupROSInterfaces();
}

void SportModeControl::SetupROSInterfaces() {
  sport_mode_ctrl_pub_ = this->create_publisher<unitree_api::msg::Request>(
      "api/sport/request", 10);
  sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10,
      std::bind(&SportModeControl::SportStateCallback, this,
                std::placeholders::_1));
}

void SportModeControl::SportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
    mode_ = msg->mode;

}