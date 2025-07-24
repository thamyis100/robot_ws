// motor_commander.hpp
#pragma once
#include <cstdint>
#include <string>

class MotorCommander {
public:
  /// Build a command like "aa c7 01 04 0F 00 03 E8 03 00 00 55"
  static std::string formatCommand(uint8_t motor_id, int32_t speed);
};
