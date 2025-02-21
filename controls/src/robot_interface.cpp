#include "robot_interface.hpp"
#include <stdexcept>

RobotInterface::RobotInterface(CANHandler& canHandler)
{
    // Create 7 motors with IDs 0 through 6 - NOTE THE NEGATIVE 1's NEED TO BE CHANGED TO TORQUE CONSTANTS
    
    // Joint 0 - MG8015 - Base Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(0), canHandler, 9, -1);
    // Joint 1 - MG8015 - Mid Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(1), canHandler, 9, -1);
    // Joint 2 - MG8008 - Brachium Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(2), canHandler, 9, -1);
    // Joint 3 - MG8008 - Elbow Flexor
    m_motors.emplace_back(static_cast<uint8_t>(3), canHandler, 9, -1);
    // Joint 4 - MG4010 - Forearm Rotator
    m_motors.emplace_back(static_cast<uint8_t>(4), canHandler, 10, -1);
    // Joint 5 - MG4005 - Wrist Differential #1 
    m_motors.emplace_back(static_cast<uint8_t>(5), canHandler, 10, -1);
    // Joint 6 - MG4005 - Wrist Differential #2
    m_motors.emplace_back(static_cast<uint8_t>(6), canHandler, 10, -1);
}

Motor& RobotInterface::getMotor(int jointIndex)
{
    if (jointIndex < 0 || jointIndex > 6) {
        throw std::out_of_range("Invalid motor index. Must be 0..6");
    }
    return m_motors.at(jointIndex);
}

void RobotInterface::updateAll()
{
    for (auto& motor : m_motors) {
        motor.update();
    }
}
