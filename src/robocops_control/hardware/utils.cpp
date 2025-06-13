#include "utils.hpp"
#include <cmath> // For M_PI constant

/**
 * @brief Converts angular velocity from radians per second (rad/s) to revolutions per minute (RPM).
 *
 * This conversion is based on the relation:
 *   1 revolution = 2π radians,
 *   1 minute = 60 seconds.
 *
 * @param rad_per_sec Angular velocity in radians per second.
 * @return Equivalent angular velocity in revolutions per minute (RPM).
 */
double rad_per_sec_to_rpm(double rad_per_sec)
{
    return (rad_per_sec * 60) / (2 * M_PI);
}

/**
 * @brief Converts angular velocity from revolutions per minute (RPM) to radians per second (rad/s).
 *
 * This conversion is based on the relation:
 *   1 revolution = 2π radians,
 *   1 minute = 60 seconds.
 *
 * @param rpm Angular velocity in revolutions per minute.
 * @return Equivalent angular velocity in radians per second.
 */
double rpm_to_rad_per_sec(double rpm)
{
    return (2 * M_PI) * rpm / 60;
}
