#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include <string>

class Motor
{
public:
    // Example: turn motor on/off
    void motorOn(uint8_t id);
    void motorOff(uint8_t id);

    // Example: set torque
    void setTorque(uint8_t id, int16_t torqueVal);

    // ... More methods to follow
private:
    // Possibly a CAN socket descriptor or handle
    // int canSocket;
};

#endif
