#include "controller_node/controller_node.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <chrono>

using namespace std::chrono_literals;
using control::JoystickController;

JoystickController::JoystickController()
: Node("joystick_controller")
{
  // 1) Declare parameters for device path and poll rate
  device_path_ = declare_parameter<std::string>("device", "/dev/input/js0");
  poll_rate_   = declare_parameter<double>("poll_rate", 100.0);

  // 2) Open joystick device in non-blocking mode
  fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to open %s", device_path_.c_str());
    throw std::runtime_error("Cannot open joystick device");
  }

  // 3) Query number of axes and buttons
  unsigned char na=0, nb=0;
  ioctl(fd_, JSIOCGAXES,   &na);
  ioctl(fd_, JSIOCGBUTTONS,&nb);
  axes_.assign(na, 0.0f);
  buttons_.assign(nb, 0);

    // 4) Create publishers
  activate_pub_ = create_publisher<std_msgs::msg::Bool>("activate", 10);
  twist_pub_    = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10);

  // 5) Poll timer: read JS events at poll_rate_
  auto poll_period = std::chrono::duration<double>(1.0 / poll_rate_);
  poll_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(poll_period),
    std::bind(&JoystickController::pollHandler, this)
  );

  // 6) Twist timer: every 10 ms send /cmd_vel if applicable
  twist_timer_ = create_wall_timer(
    10ms,
    std::bind(&JoystickController::twistTimerHandler, this)
  );

  RCLCPP_INFO(get_logger(),
              "JoystickController initialized on %s (axes=%u, buttons=%u)",
              device_path_.c_str(), na, nb);
}

void JoystickController::pollHandler()
{
  struct js_event e;
  // Read all pending events
  while (true) {
    ssize_t n = ::read(fd_, &e, sizeof(e));
    if (n != sizeof(e)) break;
    if (e.type & JS_EVENT_INIT) continue;

    if ((e.type & JS_EVENT_AXIS) && e.number < axes_.size()) {
      axes_[e.number] = e.value / 32767.0f;
    }
    else if ((e.type & JS_EVENT_BUTTON) && e.number < buttons_.size()) {
      buttons_[e.number] = e.value;
    }
  }

  // Edge‐detect Y button to toggle activate_state_
  bool y = (buttons_[BUTTON_Y] != 0);
  if (y && !prev_y_pressed_) {
    activate_state_ = !activate_state_;
    std_msgs::msg::Bool msg;
    msg.data = activate_state_;
    activate_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "Y pressed → activate %s",
                activate_state_ ? "true" : "false");
  }
  prev_y_pressed_ = y;
}

void JoystickController::twistTimerHandler()
{
  // A button held → use left stick
  if (buttons_[BUTTON_A]) {
    geometry_msgs::msg::Twist t;
    t.linear.x  = axes_[AXIS_LX] * 500;
    t.linear.y  = -axes_[AXIS_LY] * 500;
    t.angular.z = 0.0;
    twist_pub_->publish(t);
    return;
  }
  // Else if LT pressed → use right stick
  if (axes_[AXIS_LT] > LT_THRESHOLD) {
    geometry_msgs::msg::Twist t;
    t.linear.x  = 0.0;
    t.linear.y  = 0.0;
    t.angular.z = axes_[AXIS_RX] * 500;
    twist_pub_->publish(t);
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::JoystickController>());
  rclcpp::shutdown();
  return 0;
}
