/**
 * @file trajectory_follow_sample.cpp
 * @brief
 * @date 04-09-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include "unitree_sport_mode/trajectory_follow_sample.hpp"

TrajectoryFollow::TrajectoryFollow() : Node("TrajectoryFollow") {
  SetupROSInterfaces();
  t_ = -1;
}

void TrajectoryFollow::SetupROSInterfaces() {
  state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10,
      std::bind(&TrajectoryFollow::SportStateCallback, this,
                std::placeholders::_1));
  req_pub_ = this->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(dt_ * 1000)),
      std::bind(&TrajectoryFollow::TimerCallback, this));
}

void TrajectoryFollow::TimerCallback() {
  t_ += dt_;
  if (t_ > 0) {
    double time_seg = 0.2;
    double time_temp = t_ - time_seg;

    std::vector<PathPoint> path;

    for (int i = 0; i < 30; i++) {
      PathPoint path_point_tmp;
      time_temp += time_seg;
      // Tacking a sin path in x direction
      // The path is respect to the initial coordinate system
      float px_local = scalar_ * sin(scalar_ * time_temp);
      float py_local = 0;
      float yaw_local = 0.;
      float vx_local = scalar_ * cos(scalar_ * time_temp);
      float vy_local = 0;
      float vyaw_local = 0.;

      // Convert trajectory commands to the initial coordinate system
      path_point_tmp.timeFromStart = i * time_seg;
      path_point_tmp.x = px_local * cos(yaw0_) - py_local * sin(yaw0_) + px0_;
      path_point_tmp.y = px_local * sin(yaw0_) + py_local * cos(yaw0_) + py0_;
      path_point_tmp.yaw = yaw_local + yaw0_;
      path_point_tmp.vx = vx_local * cos(yaw0_) - vy_local * sin(yaw0_);
      path_point_tmp.vy = vx_local * sin(yaw0_) + vy_local * cos(yaw0_);
      path_point_tmp.vyaw = vyaw_local;
      path.push_back(path_point_tmp);
    }

    if (ctrl_mode_ == 7) {
      sport_req_.StandUp(req_);
    } else if (ctrl_mode_ == 6) {
      sport_req_.BalanceStand(req_);
    } else if (ctrl_mode_ == 1) {
      sport_req_.TrajectoryFollow(req_, path);
    }

    // Publish request messages
    req_pub_->publish(req_);
  }
}

void TrajectoryFollow::SportStateCallback(
    unitree_go::msg::SportModeState::SharedPtr msg) {
  // Get current position of robot when t<0
  // This position is used as the initial coordinate system

  if (t_ < 0) {
    // Get initial position
    px0_ = msg->position[0];
    py0_ = msg->position[1];
    yaw0_ = msg->imu_state.rpy[2];
    RCLCPP_INFO(this->get_logger(), "Position -- x: %f; y: %f; yaw: %f;", px0_,
                py0_, yaw0_);
  }
  // Get Control Mode
  ctrl_mode_ = msg->mode;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);                            // Initialize rclcpp
  rclcpp::spin(std::make_shared<TrajectoryFollow>());  // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}