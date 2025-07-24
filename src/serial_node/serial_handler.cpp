#include "serial_node/serial_handler.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>

SerialHandler::SerialHandler(const std::string &port_name, int baud_rate, DataCallback callback)
    : serial_(io_, port_name), data_callback_(callback), running_(true)
{
    reader_thread_ = std::thread([this]()
                                 { this->readLoop(); });
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    std::cout << "Serial port " << port_name << " opened successfully.\n";
}

SerialHandler::~SerialHandler()
{
    running_ = false;
    if (serial_.is_open())
    {
        serial_.close();
    }
    if (reader_thread_.joinable())
    {
        reader_thread_.join();
    }
}

void SerialHandler::readLoop()
{
    boost::asio::streambuf buf;
    while (running_)
    {
        boost::system::error_code ec;
        read_until(serial_, buf, '\n', ec);
        if (ec)
        {
            if (running_)
            {
                std::cerr << "Read error: " << ec.message() << std::endl;
            }
            continue;
        }
        std::istream is(&buf);
        std::string line;
        std::getline(is, line);
        if (!line.empty())
        {
            data_callback_(line);
        }
    }
}

void SerialHandler::sendCommand(const std::string &cmd)
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
        boost::asio::write(serial_, boost::asio::buffer(bytes));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Write error: " << e.what() << std::endl;
    }
}