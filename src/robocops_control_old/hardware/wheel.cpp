#include "wheel.hpp"

/**
 * @brief Default constructor for the Wheel class.
 *
 * Initializes the Wheel without assigning a name.
 */
Wheel::Wheel()
{
}

/**
 * @brief Constructor that initializes the Wheel with a name.
 *
 * @param wheel_name The name to assign to the wheel.
 */
Wheel::Wheel(const std::string &wheel_name)
{
    setup(wheel_name);
}

/**
 * @brief Sets the name of the wheel.
 *
 * This is useful when the wheel object is default-constructed and later needs to be initialized.
 *
 * @param wheel_name The name to assign to the wheel.
 */
void Wheel::setup(const std::string &wheel_name)
{
    name = wheel_name;
}
