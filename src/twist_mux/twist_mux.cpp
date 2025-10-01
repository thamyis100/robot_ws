#include "twist_mux/twist_mux.hpp"
#include <algorithm>
#include <cmath> // for std::isfinite

namespace robot {

TwistMux::TwistMux(const rclcpp::NodeOptions & options)
: Node("twist_mux", options)
{
  teleop_topic_ = this->declare_parameter<std::string>("teleop_topic", "/cmd_vel_teleop");
  nav_topic_ = this->declare_parameter<std::string>("nav_topic", "/cmd_vel_nav");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel");
  timeout_sec_ = this->declare_parameter<double>("timeout_sec", 0.5);
  publish_rate_ = this->declare_parameter<int>("publish_rate", 20);
  publish_zero_on_timeout_ = this->declare_parameter<bool>("publish_zero_on_timeout", false);

  publisher_ = this->create_publisher<Twist>(output_topic_, 10);

  teleop_sub_ = this->create_subscription<Twist>(
    teleop_topic_, 10, std::bind(&TwistMux::teleop_cb, this, std::placeholders::_1));

  nav_sub_ = this->create_subscription<Twist>(
    nav_topic_, 10, std::bind(&TwistMux::nav_cb, this, std::placeholders::_1));

  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, publish_rate_)));
  timer_ = this->create_wall_timer(period, std::bind(&TwistMux::timer_cb, this));

  RCLCPP_INFO(this->get_logger(), "twist_mux started. teleop: '%s' (higher priority), nav: '%s', output: '%s'",
    teleop_topic_.c_str(), nav_topic_.c_str(), output_topic_.c_str());
}

void TwistMux::teleop_cb(const Twist::SharedPtr msg)
{
  last_teleop_ = *msg;
  teleop_has_msg_ = true;
  last_teleop_time_ = this->now();
}

void TwistMux::nav_cb(const Twist::SharedPtr msg)
{
  last_nav_ = *msg;
  nav_has_msg_ = true;
  last_nav_time_ = this->now();
}

void TwistMux::timer_cb()
{
  rclcpp::Time now = this->now();

  // Teleop has higher priority
  if (teleop_has_msg_) {
    double age = (now - last_teleop_time_).seconds();
    if (age <= timeout_sec_) {
      publisher_->publish(last_teleop_);
      return;
    }
  }

  // Fallback to nav
  if (nav_has_msg_) {
    double age = (now - last_nav_time_).seconds();
    if (age <= timeout_sec_) {
      // scale nav twist by 1000 for x,y and z angular before publishing
      Twist scaled = last_nav_;

      // defensive: if values are NaN/Inf, treat as zero
      auto safe_val = [](double v) {
        return std::isfinite(v) ? v : 0.0;
      };

      scaled.linear.x  = safe_val(scaled.linear.x)  * 1000.0;
      scaled.linear.y  = safe_val(scaled.linear.y)  * 1000.0;
      // leave linear.z alone (commonly unused)
      scaled.angular.z = safe_val(scaled.angular.z) * 1000.0;
      // keep other angular components as-is (or set to 0)
      publisher_->publish(scaled);
      return;
    }
  }

  // Nothing active: optionally publish zero twist
  if (publish_zero_on_timeout_) {
    Twist zero;
    zero.linear.x = 0.0;
    zero.linear.y = 0.0;
    zero.linear.z = 0.0;
    zero.angular.x = 0.0;
    zero.angular.y = 0.0;
    zero.angular.z = 0.0;
    publisher_->publish(zero);
  }
}

} // namespace robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robot::TwistMux>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
