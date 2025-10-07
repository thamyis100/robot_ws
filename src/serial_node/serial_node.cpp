#include "serial_node/serial_node.hpp"
#include <chrono>

SerialNode::SerialNode()
    : Node("serial_node")
{
    port_name_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baudrate", 2000000);
    wheel_radius_ = declare_parameter("wheel_radius", 1);
    base_length_ = declare_parameter("base_length", 0.3);
    base_width_ = declare_parameter("base_width", 0.2);

    pub_ = create_publisher<std_msgs::msg::String>("serial/in", 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SerialNode::twistCallback, this, std::placeholders::_1));
    bool_sub_ = create_subscription<std_msgs::msg::Bool>(
        "activate", 10,
        std::bind(&SerialNode::boolCallback, this, std::placeholders::_1));

    kinematics_ = std::make_unique<MecanumKinematics>(wheel_radius_, base_length_, base_width_);
    commander_ = std::make_unique<MotorCommander>();
    serial_handler_ = std::make_unique<SerialHandler>(
        port_name_, baud_rate_,
        [this](const std::string &data)
        {
            auto msg = std_msgs::msg::String();
            msg.data = data;
            pub_->publish(msg);
        });
}

SerialNode::~SerialNode()
{
}

void SerialNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // serial_handler_->sendCommand("aa c7 01 04 0f 00 03 e8 03 00 00 55");
    // serial_handler_->sendCommand("aa c8 01 06 2b 40 60 00 0f 01 00 00 55");

    auto speeds = kinematics_->computeSpeeds(msg->linear.y, -msg->linear.x, msg->angular.z);
    // 2) Convert to int16_t (scale as needed)
    std::array<int16_t, 4> si;
    for (size_t i = 0; i < 4; ++i)
    {
        // e.g. multiply by 1, or clamp, depending on your units
         double v = speeds[i];            // scale here if needed: speeds[i] * scale
         if (v > 1000.0) v = 1000.0;
         else if (v < -1000.0) v = -1000.0;
         si[i] = static_cast<int16_t>(v);
    }
    // 3) Send one command per motor, 1ms apart
    for (uint8_t motor = 1; motor <= 4; ++motor)
    {
        std::string cmd = commander_->formatCommand(motor, si[motor - 1]);

        RCLCPP_INFO(get_logger(),
                    "Motor %d → Command: %s",
                    motor, cmd.c_str());

        serial_handler_->sendCommand(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return;
}

void SerialNode::boolCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
    {
        RCLCPP_DEBUG(get_logger(),
                     "Y pressed → activate %s",
                     msg->data ? "true" : "false");
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
    RCLCPP_DEBUG(get_logger(),
                 "Y pressed → activate %s",
                 msg->data ? "true" : "false");

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