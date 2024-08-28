/**
 * @file read_sport_mode_state.hpp
 * @brief 
 * @date 28-08-2024
 * 
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef READ_SPORT_MODE_STATE_HPP
#define READ_SPORT_MODE_STATE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"

class SportModeStateReader : public rclcpp::Node {
 public:
  SportModeStateReader();

private:
float foot_pos_[12];
float foot_vel_[12];

// Subscriber
rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;

// Callback
void SportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
};



#endif /* READ_SPORT_MODE_STATE_HPP */
