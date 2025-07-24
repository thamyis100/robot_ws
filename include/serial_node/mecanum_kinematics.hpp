#ifndef MECANUM_KINEMATICS_HPP
#define MECANUM_KINEMATICS_HPP

#include <vector>

class MecanumKinematics {
public:
    MecanumKinematics(double wheel_radius, double base_length, double base_width);
    std::vector<double> computeSpeeds(double vx, double vy, double omega);

private:
    double wheel_radius_;
    double base_length_;
    double base_width_;
};

#endif // MECANUM_KINEMATICS_HPP