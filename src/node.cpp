#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <boost/asio.hpp>
#include <iomanip> // for std::hex, setw, setfill
#include <thread>
#include <mutex>
#include <vector>
#include <sstream>

using boost::asio::io_service;
using boost::asio::read_until;
using boost::asio::serial_port;
using boost::asio::streambuf;
using boost::asio::write;

class SerialNode : public rclcpp::Node
{
public:
  SerialNode()
      : Node("serial_node")
  {
    // 1) Declare & read parameters
    port_name_ = declare_parameter("port", std::string("/dev/ttyUSB0"));
    baud_rate_ = declare_parameter("baudrate", 200000);
    wheel_radius_ = declare_parameter("wheel_radius", 0.05);
    base_length_ = declare_parameter("base_length", 0.3);
    base_width_ = declare_parameter("base_width", 0.2);

    // 2) Publisher for incoming serial (unchanged)
    pub_ = create_publisher<std_msgs::msg::String>("serial/in", 10);

    // 3) Subscriber to cmd_vel (x=linear.x, y=linear.y, z=angular.z)
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SerialNode::twistCallback, this, std::placeholders::_1));

    bool_sub_ = create_subscription<std_msgs::msg::Bool>(
        "activate", 10,
        std::bind(&SerialNode::boolCallback, this, std::placeholders::_1));

    // 4) Open serial port
    try
    {
      io_ = std::make_shared<io_service>();
      serial_ = std::make_shared<serial_port>(*io_, port_name_);
      serial_->set_option(serial_port::baud_rate(baud_rate_));
      RCLCPP_INFO(get_logger(),
                  "Serial open: %s @ %d baud",
                  port_name_.c_str(), baud_rate_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open %s: %s",
                   port_name_.c_str(), e.what());
      return;
    }

    // 5) Start read thread (unchanged)
    reader_thread_ = std::thread([this]()
                                 { this->readLoop(); });
  }

  ~SerialNode()
  {
    if (serial_ && serial_->is_open())
    {
      serial_->cancel();
      serial_->close();
    }
    if (reader_thread_.joinable())
    {
      reader_thread_.join();
    }
    if (io_)
    {
      io_->stop();
    }
  }

private:
  void readLoop()
  {
    streambuf buf;
    while (rclcpp::ok())
    {
      try
      {
        read_until(*serial_, buf, '\n');
        std::istream is(&buf);
        std::string line;
        std::getline(is, line);
        if (!line.empty())
        {
          auto msg = std_msgs::msg::String();
          msg.data = line;
          pub_->publish(msg);
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "Serial read error: %s", e.what());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  // data formatter
  std::string formatMotorCommand(uint8_t motor_id, int16_t speed)
  {
    // clamp speed to int16 range if you want:
    // speed = std::max<int16_t>(std::numeric_limits<int16_t>::min(),
    //          std::min<int16_t>(std::numeric_limits<int16_t>::max(), speed));

    uint8_t hi = (speed >> 8) & 0xFF;
    uint8_t lo = speed & 0xFF;

    std::ostringstream oss;
    oss << std::hex << std::setfill('0')
        // preamble
        << std::setw(2) << int(0xAA) << ' '
        << std::setw(2) << int(0xC7) << ' '
        // motor ID and command code
        << std::setw(2) << int(motor_id) << ' '
        << std::setw(2) << int(0x04) << ' '
        // speed bytes
        << std::setw(2) << int(hi) << ' '
        << std::setw(2) << int(lo);

    oss.str() += '\n';
    return oss.str();
  }

  /// Inverse kinematics for mecanum wheels
  std::vector<double> computeMecanumSpeeds(double vx, double vy, double omega)
  {
    double L = base_length_;
    double W = base_width_;
    double R = wheel_radius_;
    // Order: Front‑Left, Front‑Right, Rear‑Left, Rear‑Right
    return {
        (vx - vy - (L + W) * omega) / R,
        (vx + vy + (L + W) * omega) / R,
        (vx + vy - (L + W) * omega) / R,
        (vx - vy + (L + W) * omega) / R,
    };
  }

  // Synchronous send helper
  void sendCommand(const std::string &cmd)
  {
    std::vector<uint8_t> bytes;
    std::istringstream iss(cmd);
    std::string byteStr;

    try
    {
      while (iss >> byteStr)
      {
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteStr, nullptr, 16));
        bytes.push_back(byte);
      }

      std::lock_guard<std::mutex> lock(serial_mutex_);
      boost::asio::write(*serial_, boost::asio::buffer(bytes));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "sendCommand error: %s", e.what());
    }
  }

  // receiveing velocity data
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 1) Compute wheel speeds (as before)
    auto speeds = computeMecanumSpeeds(
        msg->linear.x, msg->linear.y, msg->angular.z);

    // 2) Convert to int16_t (scale as needed)
    std::array<int16_t, 4> si;
    for (size_t i = 0; i < 4; ++i)
    {
      // e.g. multiply by 1000, or clamp, depending on your units
      si[i] = static_cast<int16_t>(speeds[i] * 1000);
    }

    // 3) Send one command per motor, 25ms apart
    std::lock_guard<std::mutex> lk(serial_mutex_);
    for (uint8_t motor = 1; motor <= 4; ++motor)
    {
      auto cmd = formatMotorCommand(motor, si[motor - 1]);
      cmd += "";

      try
      {

        boost::asio::write(*serial_, boost::asio::buffer(cmd));
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "Serial write error: %s", e.what());
      }
      // pause 25 ms before next motor
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
  }

  // receiveing velocity data
  void boolCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
    {
      // disable motor
      sendCommand("aa c8 01 06 2b 40 60 00 0f 01 00 00 55");
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      sendCommand("aa c8 02 06 2b 40 60 00 0f 01 00 00 55");
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      sendCommand("aa c8 03 06 2b 40 60 00 0f 01 00 00 55");
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      sendCommand("aa c8 04 06 2b 40 60 00 0f 01 00 00 55");
      std::this_thread::sleep_for(std::chrono::milliseconds(25));

      return;
    }

    // usbcan setup
    sendCommand("aa 55 12 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 14");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    // motor mode
    sendCommand("aa c8 01 06 2f 60 60 00 03 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    sendCommand("aa c8 02 06 2f 60 60 00 03 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    sendCommand("aa c8 03 06 2f 60 60 00 03 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    sendCommand("aa c8 04 06 2f 60 60 00 03 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    // operation_state
    sendCommand("aa c2 00 00 01 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    // enable motor
    sendCommand("aa c8 01 06 2b 40 60 00 0f 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    sendCommand("aa c8 02 06 2b 40 60 00 0f 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    sendCommand("aa c8 03 06 2b 40 60 00 0f 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    sendCommand("aa c8 04 06 2b 40 60 00 0f 00 00 00 55");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  // Parameters
  std::string port_name_;
  int baud_rate_;
  double wheel_radius_, base_length_, base_width_;

  // Boost.Asio
  std::shared_ptr<io_service> io_;
  std::shared_ptr<serial_port> serial_;
  std::thread reader_thread_;
  std::mutex serial_mutex_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
