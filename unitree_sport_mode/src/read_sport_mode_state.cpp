/**
 * @file read_sport_mode_state.cpp
 * @brief
 * @date 28-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include "unitree_sport_mode/read_sport_mode_state.hpp"

#define DISPLAY_FOOT_DATA 1

SportModeStateReader::SportModeStateReader() : Node("SportModeStateReader") {
  sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10,
      std::bind(&SportModeStateReader::SportStateCallback, this,
                std::placeholders::_1));
}

void SportModeStateReader::SportStateCallback(
    const unitree_go::msg::SportModeState::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Motion mode: %d", msg->mode);
  RCLCPP_INFO(this->get_logger(),
              "Gait state -- gait type: %d; raise height: %f", msg->gait_type,
              msg->foot_raise_height);
  RCLCPP_INFO(
      this->get_logger(), "Position -- x: %f; y: %f; z: %f; body height: %f",
      msg->position[0], msg->position[1], msg->position[2], msg->body_height);
  RCLCPP_INFO(this->get_logger(), "Velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
              msg->velocity[0], msg->velocity[1], msg->velocity[2],
              msg->yaw_speed);

  if (DISPLAY_FOOT_DATA) {
    for (int i = 0; i < 12; i++) {
      foot_pos_[i] = msg->foot_position_body[i];
      foot_vel_[i] = msg->foot_speed_body[i];
    }

    RCLCPP_INFO(this->get_logger(),
                "Foot position and velocity relative to body -- num: %d; x: %f; "
                "y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                0, foot_pos_[0], foot_pos_[1], foot_pos_[2], foot_vel_[0],
                foot_vel_[1], foot_vel_[2]);
    RCLCPP_INFO(this->get_logger(),
                "Foot position and velocity relative to body -- num: %d; x: %f; "
                "y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                1, foot_pos_[3], foot_pos_[4], foot_pos_[5], foot_vel_[3],
                foot_vel_[4], foot_vel_[5]);
    RCLCPP_INFO(this->get_logger(),
                "Foot position and velocity relative to body -- num: %d; x: %f; "
                "y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                2, foot_pos_[6], foot_pos_[7], foot_pos_[8], foot_vel_[6],
                foot_vel_[7], foot_vel_[8]);
    RCLCPP_INFO(this->get_logger(),
                "Foot position and velocity relative to body -- num: %d; x: %f; "
                "y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                3, foot_pos_[9], foot_pos_[10], foot_pos_[11], foot_vel_[9],
                foot_vel_[10], foot_vel_[11]);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SportModeStateReader>());
  rclcpp::shutdown();
  return 0;
}