#include "utils.hpp"
#include <cmath> // For M_PI constant

// Function to convert radians per second (rad/s) to revolutions per minute (RPM)
double rad_per_sec_to_rpm(double rad_per_sec)
{
    return (rad_per_sec * 60) / (2 * M_PI);
}

// Function to convert revolutions per minute (RPM) to radians per second (rad/s)
double rpm_to_rad_per_sec(double rpm)
{
    return (2 * M_PI) * rpm / 60;
}
