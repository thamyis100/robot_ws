#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace control {

class JoystickController : public rclcpp::Node
{
public:
  JoystickController();

private:
  void pollHandler();        // read joystick events
  void twistTimerHandler();  // publish /cmd_vel periodically

  int                       fd_;           // joystick file descriptor
  std::vector<float>        axes_;         // axis values
  std::vector<int32_t>      buttons_;      // button states

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      activate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr twist_timer_;

  bool prev_y_pressed_ = false;
  bool activate_state_ = false;

  std::string device_path_;
  double      poll_rate_;

  // mapping indices
  static constexpr int BUTTON_Y = 3;
  static constexpr int BUTTON_A = 0;
  static constexpr int AXIS_LX   = 0;
  static constexpr int AXIS_LY   = 1;
  static constexpr int AXIS_RX   = 3;
  static constexpr int AXIS_RY   = 4;
  static constexpr int AXIS_LT   = 2;
  static constexpr double LT_THRESHOLD = 0.5;
};

} // namespace control
