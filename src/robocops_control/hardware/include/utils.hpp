#ifndef UTILS_HPP
#define UTILS_HPP

/**
 * @brief Converts angular velocity from radians per second (rad/s) to revolutions per minute (RPM).
 *
 * @param rad_per_sec Angular velocity in radians per second.
 * @return Angular velocity in revolutions per minute (RPM).
 */
double rad_per_sec_to_rpm(double rad_per_sec);

/**
 * @brief Converts angular velocity from revolutions per minute (RPM) to radians per second (rad/s).
 *
 * @param rpm Angular velocity in revolutions per minute.
 * @return Angular velocity in radians per second (rad/s).
 */
double rpm_to_rad_per_sec(double rpm);

#endif // UTILS_HPP
