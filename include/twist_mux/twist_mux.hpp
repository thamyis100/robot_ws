#ifndef ROBOT_TWIST_MUX__TWIST_MUX_HPP_
#define ROBOT_TWIST_MUX__TWIST_MUX_HPP_


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace robot {


class TwistMux : public rclcpp::Node
{
public:
explicit TwistMux(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


private:
using Twist = geometry_msgs::msg::Twist;


void teleop_cb(const Twist::SharedPtr msg);
void nav_cb(const Twist::SharedPtr msg);
void timer_cb();


// Parameters
std::string teleop_topic_;
std::string nav_topic_;
std::string output_topic_;
double timeout_sec_ = 0.5;
int publish_rate_ = 20;
bool publish_zero_on_timeout_ = false;


// ROS interfaces
rclcpp::Publisher<Twist>::SharedPtr publisher_;
rclcpp::Subscription<Twist>::SharedPtr teleop_sub_;
rclcpp::Subscription<Twist>::SharedPtr nav_sub_;
rclcpp::TimerBase::SharedPtr timer_;


// State
Twist last_teleop_;
Twist last_nav_;
bool teleop_has_msg_ = false;
bool nav_has_msg_ = false;
rclcpp::Time last_teleop_time_;
rclcpp::Time last_nav_time_;
};


} // namespace robot


#endif // ROBOT_TWIST_MUX__TWIST_MUX_HPP_