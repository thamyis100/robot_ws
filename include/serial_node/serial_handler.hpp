#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <boost/asio.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <functional>

class SerialHandler {
public:
    using DataCallback = std::function<void(const std::string&)>;
    SerialHandler(const std::string& port_name, int baud_rate, DataCallback callback);
    ~SerialHandler();

    void sendCommand(const std::string& cmd);

private:
    void readLoop();

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::thread reader_thread_;
    std::mutex serial_mutex_;
    DataCallback data_callback_;
    bool running_;
};

#endif // SERIAL_HANDLER_HPP