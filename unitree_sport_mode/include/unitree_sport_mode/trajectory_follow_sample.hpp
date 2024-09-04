/**
 * @file trajectory_follow_sample.hpp
 * @brief
 * @date 04-09-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef TRAJECTORY_FOLLOW_SAMPLE_HPP
#define TRAJECTORY_FOLLOW_SAMPLE_HPP

#include <chrono>
#include <memory>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_ros2_client/common/ros2_sport_client.h"

class TrajectoryFollow : public rclcpp::Node {
 public:
  TrajectoryFollow();
  void SetupROSInterfaces();
  void SportStateCallback(unitree_go::msg::SportModeState::SharedPtr msg);
  void TimerCallback();

 private:
  // Subscriber
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;

  // Publisher
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  unitree_api::msg::Request req_;  // Unitree Go2 ROS2 request message
  SportClient sport_req_;

  double t_;           // running time count
  double dt_ = 0.002;  // control time step
  double scalar_ = 0.2;

  double px0_ = 0;   // initial x position
  double py0_ = 0;   // initial y position
  double yaw0_ = 0;  // initial yaw angle
  int ctrl_mode_;
};

#endif /* TRAJECTORY_FOLLOW_SAMPLE_HPP */