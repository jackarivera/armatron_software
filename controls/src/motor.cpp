#include "motor.hpp"
#include <iostream>

// For actual SocketCAN usage, we'll include <linux/can.h>, <linux/can/raw.h>, etc.
// #include <sys/socket.h>
// #include <net/if.h>
// #include <sys/ioctl.h>

void Motor::motorOn(uint8_t id)
{
    // Placeholder: real implementation will construct
    // a CAN frame with the "motor on" command per your docs.
    std::cout << "[Motor] motorOn called for ID=" << (int)id << std::endl;
}

void Motor::motorOff(uint8_t id)
{
    // Placeholder
    std::cout << "[Motor] motorOff called for ID=" << (int)id << std::endl;
}

void Motor::setTorque(uint8_t id, int16_t torqueVal)
{
    // Placeholder
    std::cout << "[Motor] setTorque: ID=" << (int)id 
              << ", torqueVal=" << torqueVal << std::endl;
}
