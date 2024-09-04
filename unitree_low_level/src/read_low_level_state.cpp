/**
 * @file read_low_level_state.cpp
 * @brief
 * @date 30-08-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include "unitree_low_level/read_low_level_state.hpp"

#define INFO_IMU 1
#define INFO_MOTOR 1
#define INFO_FOOT_FORCE 1
#define INFO_BATTERY 1

LowLevelReader::LowLevelReader() : Node("LowLevelReader") {
  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "lowstate", 10,
      std::bind(&LowLevelReader::LowLevelCallback, this,
                std::placeholders::_1));
}

void LowLevelReader::LowLevelCallback(
    const unitree_go::msg::LowState::SharedPtr msg) {
  if (INFO_IMU) {
    // Info IMU states
    // RPY euler angle(ZYX order respected to body frame)
    // Quaternion
    // Gyroscope (raw data)
    // Accelerometer (raw data)
    imu_ = msg->imu_state;

    RCLCPP_INFO(this->get_logger(),
                "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
                imu_.rpy[1], imu_.rpy[2]);
    RCLCPP_INFO(this->get_logger(),
                "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
                imu_.quaternion[3]);
    RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
    RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                imu_.accelerometer[0], imu_.accelerometer[1],
                imu_.accelerometer[2]);
  }

  if (INFO_MOTOR) {
    // Info motor states
    // q: angluar (rad)
    // dq: angluar velocity (rad/s)
    // ddq: angluar acceleration (rad/(s^2))
    // tau_est: Estimated external torque

    for (int i = 0; i < 12; i++) {
      motor_[i] = msg->motor_state[i];
      RCLCPP_INFO(this->get_logger(),
                  "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f", i,
                  motor_[i].q, motor_[i].dq, motor_[i].ddq, motor_[i].tau_est);
    }
  }

  if (INFO_FOOT_FORCE) {
    // Info foot force value (int not true value)
    for (int i = 0; i < 4; i++) {
      foot_force_[i] = msg->foot_force[i];
      foot_force_est_[i] = msg->foot_force_est[i];
    }

    RCLCPP_INFO(this->get_logger(),
                "Foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                foot_force_[0], foot_force_[1], foot_force_[2], foot_force_[3]);
    RCLCPP_INFO(
        this->get_logger(),
        "Estimated foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
        foot_force_est_[0], foot_force_est_[1], foot_force_est_[2],
        foot_force_est_[3]);
  }

  if (INFO_BATTERY) {
    // Info battery states
    // battery current
    // battery voltage
    battery_current_ = msg->power_a;
    battery_voltage_ = msg->power_v;

    RCLCPP_INFO(this->get_logger(), "Battery state -- current: %f; voltage: %f",
                battery_current_, battery_voltage_);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowLevelReader>());
  rclcpp::shutdown();
  return 0;
}