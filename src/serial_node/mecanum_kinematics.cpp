#include "serial_node/mecanum_kinematics.hpp"

MecanumKinematics::MecanumKinematics(double wheel_radius, double base_length, double base_width)
    : wheel_radius_(wheel_radius), base_length_(base_length), base_width_(base_width)
{
}

std::vector<double> MecanumKinematics::computeSpeeds(double vx, double vy, double omega) {
    double L = base_length_;
    double W = base_width_;
    double R = wheel_radius_;
    return {
        (vx - vy + (L + W) * omega) / R,
        (vx + vy + (L + W) * omega) / R,
        (-vx + vy + (L + W) * omega) / R,
        (-vx - vy + (L + W) * omega) / R,

        // (vx + vy + (L + W) * omega) / R,  // Motor 1 (Front-Right)
        // (vx - vy - (L + W) * omega) / R,  // Motor 2 (Front-Left)
        // (vx + vy - (L + W) * omega) / R,  // Motor 3 (Rear-Left)
        // (vx - vy + (L + W) * omega) / R,  // Motor 4 (Rear-Right)
    };
}