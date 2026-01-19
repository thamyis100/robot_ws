#include "serial_node/serial_node.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

SerialNode::SerialNode()
  : Node("serial_node")
{
  // --- Parameters ---
  port_name_    = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_rate_    = declare_parameter<int>("baudrate", 2000000);

  wheel_radius_ = declare_parameter<double>("wheel_radius", 1.0);
  base_length_  = declare_parameter<double>("base_length", 0.3);
  base_width_   = declare_parameter<double>("base_width", 0.2);

  // Option B smoothing
  max_pwm_         = declare_parameter<double>("max_pwm", 2000.0);
  smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.2); // 0..1

  // Per-motor direction (set -1 for reversed motor)
  // Example YAML:
  // motor_sign: [1, -1, 1, -1]
  auto ms = declare_parameter<std::vector<int64_t>>("motor_sign", {1, 1, 1, 1});
  if (ms.size() == 4) {
    for (int i = 0; i < 4; ++i) motor_sign_[i] = (ms[i] >= 0) ? 1 : -1;
  }

  // Clamp alpha
  smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);

  // --- Pub/Sub ---
  pub_ = create_publisher<std_msgs::msg::String>("serial/in", 10);

  sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SerialNode::twistCallback, this, std::placeholders::_1));

  bool_sub_ = create_subscription<std_msgs::msg::Bool>(
      "activate", 10,
      std::bind(&SerialNode::boolCallback, this, std::placeholders::_1));

  // --- Helpers ---
  kinematics_ = std::make_unique<MecanumKinematics>(wheel_radius_, base_length_, base_width_);
  commander_  = std::make_unique<MotorCommander>();

  serial_handler_ = std::make_unique<SerialHandler>(
      port_name_, baud_rate_,
      [this](const std::string &data)
      {
        auto msg = std_msgs::msg::String();
        msg.data = data;
        pub_->publish(msg);
      });
}

void SerialNode::sendFourMotors(const std::array<int16_t, 4>& pwm)
{
  for (uint8_t motor = 1; motor <= 4; ++motor)
  {
    std::string cmd = commander_->formatCommand(motor, pwm[motor - 1]);
    serial_handler_->sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void SerialNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!active_) {
    // If not active, ignore cmd_vel (or you can send zeros here if you prefer)
    return;
  }

  // Your mapping: computeSpeeds(vx, vy, wz)
  // (keep same as your original)
  auto speeds = kinematics_->computeSpeeds(-msg->linear.y, msg->linear.x, msg->angular.z);

  std::array<double, 4> target{};
  for (size_t i = 0; i < 4; ++i)
  {
    double v = speeds[i]; // apply scaling here if you want (e.g. *scale)
    if (v >  max_pwm_) v =  max_pwm_;
    if (v < -max_pwm_) v = -max_pwm_;
    target[i] = v;
  }

  // --- Option B: Exponential smoothing (EMA) per wheel ---
  // last = last + alpha*(target - last)
  for (int i = 0; i < 4; ++i)
  {
    last_pwm_[i] = last_pwm_[i] + smoothing_alpha_ * (target[i] - last_pwm_[i]);
    if (last_pwm_[i] >  max_pwm_) last_pwm_[i] =  max_pwm_;
    if (last_pwm_[i] < -max_pwm_) last_pwm_[i] = -max_pwm_;
  }

  // Convert to int16 and apply motor_sign
  std::array<int16_t, 4> out{};
  for (int i = 0; i < 4; ++i)
  {
    double v = last_pwm_[i] * static_cast<double>(motor_sign_[i]);
    if (v >  32767.0) v =  32767.0;
    if (v < -32768.0) v = -32768.0;
    out[i] = static_cast<int16_t>(v);
  }

  // Send immediately (only when receiving cmd_vel)
  sendFourMotors(out);
}

void SerialNode::boolCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  active_ = msg->data;

  if (!active_)
  {
    RCLCPP_DEBUG(get_logger(), "activate false");

    // Optional: STOP motors immediately on deactivate
    last_pwm_ = {0, 0, 0, 0};
    sendFourMotors({0, 0, 0, 0});

    serial_handler_->sendCommand("aa c8 01 06 2b 40 60 00 0f 01 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    serial_handler_->sendCommand("aa c8 02 06 2b 40 60 00 0f 01 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    serial_handler_->sendCommand("aa c8 03 06 2b 40 60 00 0f 01 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    serial_handler_->sendCommand("aa c8 04 06 2b 40 60 00 0f 01 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    return;
  }

  RCLCPP_DEBUG(get_logger(), "activate true");

  serial_handler_->sendCommand("aa 55 12 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 14");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  serial_handler_->sendCommand("aa c8 01 06 2f 60 60 00 03 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  serial_handler_->sendCommand("aa c8 02 06 2f 60 60 00 03 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  serial_handler_->sendCommand("aa c8 03 06 2f 60 60 00 03 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  serial_handler_->sendCommand("aa c8 04 06 2f 60 60 00 03 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  serial_handler_->sendCommand("aa c2 00 00 01 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  serial_handler_->sendCommand("aa c8 01 06 2b 40 60 00 0f 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  serial_handler_->sendCommand("aa c8 02 06 2b 40 60 00 0f 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  serial_handler_->sendCommand("aa c8 03 06 2b 40 60 00 0f 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  serial_handler_->sendCommand("aa c8 04 06 2b 40 60 00 0f 00 00 00 55");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNode>();
  RCLCPP_INFO(node->get_logger(), "SerialNode starting up…");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
