#include "motor.hpp"
#include <iostream>

int main()
{
    Motor motor;
    std::cout << "Starting motor control test...\n";

    // Example usage
    motor.motorOn(1);
    motor.setTorque(1, 500);
    motor.motorOff(1);

    std::cout << "Done.\n";
    return 0;
}
