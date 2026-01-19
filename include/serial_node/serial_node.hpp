#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>
#include <array>
#include <vector>

// Your project headers (keep as in your project)
#include "serial_handler.hpp"
#include "mecanum_kinematics.hpp"
#include "motor_commander.hpp"

class SerialNode : public rclcpp::Node
{
public:
  SerialNode();
  ~SerialNode() override = default;

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void boolCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void sendFourMotors(const std::array<int16_t, 4>& pwm);

private:
  // Parameters
  std::string port_name_;
  int baud_rate_;
  double wheel_radius_;
  double base_length_;
  double base_width_;

  double max_pwm_;            // clamp target PWM
  double smoothing_alpha_;    // 0..1
  std::array<int, 4> motor_sign_{ {1, 1, 1, 1} }; // per-wheel sign invert

  // ROS pub/sub
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;

  // Helpers
  std::unique_ptr<MecanumKinematics> kinematics_;
  std::unique_ptr<MotorCommander> commander_;
  std::unique_ptr<SerialHandler> serial_handler_;

  // State
  std::array<double, 4> last_pwm_{ {0, 0, 0, 0} };
  bool active_{true};
};
