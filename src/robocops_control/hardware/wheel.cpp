#include "Wheel.hpp"

Wheel::Wheel()
{
}

Wheel::Wheel(const std::string &wheel_name)
{
    setup(wheel_name);
}

void Wheel::setup(const std::string &wheel_name)
{
    name = wheel_name;
}
