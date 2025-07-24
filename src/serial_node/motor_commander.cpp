// motor_commander.cpp
#include "serial_node/motor_commander.hpp"
#include <cstdio>   // for std::snprintf
#include <cstdint>  // for uint8_t, int32_t
#include <string>

std::string MotorCommander::formatCommand(uint8_t motor_id, int32_t speed) {
    // Split speed into little‑endian bytes:
    uint8_t b0 = static_cast<uint8_t>( speed        & 0xFF );
    uint8_t b1 = static_cast<uint8_t>((speed >>  8) & 0xFF );
    uint8_t b2 = static_cast<uint8_t>((speed >> 16) & 0xFF );
    uint8_t b3 = static_cast<uint8_t>((speed >> 24) & 0xFF );

    // Stack buffer—more than enough for our ~34 chars + '\0'
    char buf[64];

    // Build the command string:
    int len = std::snprintf(
        buf, sizeof(buf),
        "aa c7 %02d 04 0F 00 03 %02X %02X %02X %02X 55",
        // motor_id as decimal 2‑digit
        static_cast<int>(motor_id),
        // speed bytes as hex
        b0, b1, b2, b3
    );

    if (len < 0) {
        // formatting error
        return std::string{};
    }

    // Construct and return a std::string of exactly 'len' chars
    return std::string(buf, static_cast<size_t>(len));
}
