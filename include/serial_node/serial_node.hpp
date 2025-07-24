#ifndef SERIAL_NODE_HPP
#define SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "serial_handler.hpp"
#include "mecanum_kinematics.hpp"
#include "motor_commander.hpp"
#include <memory>
#include <string>

class SerialNode : public rclcpp::Node {
public:
    SerialNode();
    ~SerialNode();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void boolCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // Parameters
    std::string port_name_;
    int baud_rate_;
    double wheel_radius_, base_length_, base_width_;

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;

    // Modules
    std::unique_ptr<SerialHandler> serial_handler_;
    std::unique_ptr<MecanumKinematics> kinematics_;
    std::unique_ptr<MotorCommander> commander_;

    //BOOST.ASIO
    std::mutex serial_mutex_;
};

#endif // SERIAL_NODE_HPP