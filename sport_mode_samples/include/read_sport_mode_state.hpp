/**
 * @file read_sport_mode_state.hpp
 * @author Kartheegeyan (kartheegeyan.mahandra@westonrobot.com)
 * @brief
 * @version 0.1
 * @date 2024-08-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef READ_SPORT_MODE_STATE_HPP
#define READ_SPORT_MODE_STATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"

class ReadSportModeState : public rclcpp::Node {
 public:
  ReadSportModeState(/* args */);
  ReadSportModeState();
};
ReadSportModeState : ReadSportModeState(/* args */) {}
ReadSportModeState::ReadSportModeState() {}

private:
/* data */

#endif /* READ_SPORT_MODE_STATE_HPP */