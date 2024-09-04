/**
 * @file read_low_level_state.hpp
 * @brief
 * @date 30-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef READ_LOW_LEVEL_STATE_HPP
#define READ_LOW_LEVEL_STATE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"

class LowLevelReader : public rclcpp::Node {
 public:
  LowLevelReader();

 private:
  unitree_go::msg::IMUState imu_;          // Unitree go2 IMU message
  unitree_go::msg::MotorState motor_[12];  // Unitree go2 motor state message
  int16_t foot_force_[4];                  // External contact force value (int)
  int16_t foot_force_est_[4];              // Estimated  external contact force value (int)
  float battery_voltage_;                  // Battery voltage
  float battery_current_;                  // Battery current

  // Subscriber
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

  // Callback
  void LowLevelCallback(const unitree_go::msg::LowState::SharedPtr msg);
};

#endif /* READ_LOW_LEVEL_STATE_HPP */